from __future__ import annotations

import hashlib
import json
import mimetypes
from pathlib import Path
import secrets
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlparse
from urllib.request import Request, urlopen

from h753_can_odom.mission_data_core import OutboxEvent, SCHEMA_VERSION


class MissionUploadError(RuntimeError):
    """Upload failure with enough context to choose retry or quarantine."""

    def __init__(
        self,
        message: str,
        *,
        retryable: bool = True,
        status_code: int | None = None,
    ) -> None:
        super().__init__(message)
        self.retryable = retryable
        self.status_code = status_code


def encode_multipart(
    fields: dict[str, str],
    files: list[tuple[str, Path]],
) -> tuple[bytes, str]:
    boundary = f'h753-{secrets.token_hex(16)}'
    body = bytearray()

    def append(value: str | bytes) -> None:
        body.extend(value.encode() if isinstance(value, str) else value)

    for name, value in fields.items():
        append(f'--{boundary}\r\n')
        append(
            f'Content-Disposition: form-data; name="{name}"\r\n\r\n'
        )
        append(value)
        append('\r\n')
    for field_name, path in files:
        mime_type = mimetypes.guess_type(path.name)[0]
        if mime_type is None:
            mime_type = 'application/octet-stream'
        append(f'--{boundary}\r\n')
        append(
            'Content-Disposition: form-data; '
            f'name="{field_name}"; filename="{path.name}"\r\n'
        )
        append(f'Content-Type: {mime_type}\r\n\r\n')
        append(path.read_bytes())
        append('\r\n')
    append(f'--{boundary}--\r\n')
    return bytes(body), f'multipart/form-data; boundary={boundary}'


class MissionApiClient:
    """Small dependency-free client for the server's durable mission API."""

    def __init__(
        self,
        base_url: str,
        token: str = '',
        api_key_header: str = 'X-API-Key',
        map_upload_path: str = '/api/v1/maps:upload',
        map_yaml_field: str = 'yaml_file',
        map_image_field: str = 'pgm_file',
        timeout_s: float = 5.0,
        allow_insecure_http: bool = False,
        opener: Callable[..., Any] = urlopen,
    ) -> None:
        self.base_url = base_url.rstrip('/')
        parsed = urlparse(self.base_url)
        if parsed.scheme not in ('http', 'https') or not parsed.netloc:
            raise ValueError('api_base_url must be an absolute HTTP(S) URL')
        if parsed.scheme != 'https' and not allow_insecure_http:
            raise ValueError(
                'Plain HTTP is disabled; use HTTPS or explicitly enable '
                'allow_insecure_http for an isolated development LAN'
            )
        if timeout_s <= 0.0:
            raise ValueError('timeout_s must be positive')
        if not api_key_header or not all(
            character.isalnum() or character == '-'
            for character in api_key_header
        ):
            raise ValueError('api_key_header must be a valid HTTP header name')
        if not map_upload_path.startswith('/'):
            raise ValueError('map_upload_path must start with /')
        for name, value in (
            ('map_yaml_field', map_yaml_field),
            ('map_image_field', map_image_field),
        ):
            if not value or not all(
                character.isalnum() or character == '_'
                for character in value
            ):
                raise ValueError(f'{name} must be a valid multipart field')
        self.token = token
        self.api_key_header = api_key_header
        self.map_upload_path = map_upload_path
        self.map_yaml_field = map_yaml_field
        self.map_image_field = map_image_field
        self.timeout_s = timeout_s
        self.opener = opener

    def upload_event(self, event: OutboxEvent) -> dict[str, Any]:
        event_type = event.event_type
        if event_type == 'map':
            return self._upload_map(event)
        if event_type == 'mission_start':
            payload = {
                key: event.payload.get(key)
                for key in (
                    'schema_version',
                    'mission_id',
                    'robot_id',
                    'map_id',
                    'started_at',
                )
            }
            response = self._json_request(
                'POST',
                '/api/v1/missions',
                payload,
                event.event_key,
            )
            self._require_ack_identifier(
                response,
                'mission_id',
                event.mission_id,
            )
            return response
        if event_type == 'mission_end':
            mission_id = quote(event.mission_id, safe='')
            payload = {
                key: event.payload.get(key)
                for key in ('status', 'ended_at')
                if event.payload.get(key) is not None
            }
            response = self._json_request(
                'PATCH',
                f'/api/v1/missions/{mission_id}',
                payload,
                event.event_key,
            )
            self._require_ack_identifier(
                response,
                'mission_id',
                event.mission_id,
            )
            return response
        if event_type == 'detection':
            robot_pose = event.payload.get('robot_pose') or {}
            yolo = event.payload.get('yolo') or {}
            payload = {
                key: event.payload.get(key)
                for key in (
                    'schema_version',
                    'mission_id',
                    'detection_id',
                    'detected_at',
                    'map_id',
                    'frame_id',
                    'route_end_seq',
                )
            }
            payload['robot_pose'] = {
                key: robot_pose.get(key) for key in ('x', 'y', 'yaw')
            }
            payload['yolo'] = {
                key: yolo.get(key)
                for key in (
                    'person_found',
                    'blue_person_found',
                    'nearest_distance_m',
                )
                if key in yolo
            }
            response = self._json_request(
                'POST',
                '/api/v1/detections',
                payload,
                event.event_key,
            )
            self._require_ack_identifier(
                response,
                'detection_id',
                str(event.payload['detection_id']),
            )
            return response
        if event_type == 'detection_image':
            return self._upload_detection_image(event)
        raise MissionUploadError(
            f'Unsupported outbox event: {event_type}',
            retryable=False,
        )

    def health(self) -> dict[str, Any]:
        response = self._request(
            'GET',
            '/api/v1/health',
            None,
            '',
            '',
        )
        if response.get('status') != 'ok':
            raise MissionUploadError(
                f'Unexpected health response: {response}',
                retryable=False,
            )
        if response.get('schema_version') != SCHEMA_VERSION:
            raise MissionUploadError(
                'Server schema version does not match Jetson: '
                f'{response.get("schema_version")} != {SCHEMA_VERSION}',
                retryable=False,
            )
        return response

    def upload_route_batch(
        self,
        mission_id: str,
        points: list[dict[str, Any]],
    ) -> dict[str, Any]:
        if not points:
            return {}
        payload = {
            'schema_version': SCHEMA_VERSION,
            'points': [
                {
                    key: point.get(key)
                    for key in (
                        'route_seq',
                        'stamp_ns',
                        'x',
                        'y',
                        'yaw',
                        'quality_flags',
                    )
                }
                for point in points
            ],
        }
        first = int(points[0]['route_seq'])
        last = int(points[-1]['route_seq'])
        key = f'route:{mission_id}:{first}-{last}'
        mission = quote(mission_id, safe='')
        response = self._json_request(
            'POST',
            f'/api/v1/missions/{mission}/route-points:batch',
            payload,
            key,
        )
        self._require_ack_identifier(response, 'mission_id', mission_id)
        return response

    def _upload_map(self, event: OutboxEvent) -> dict[str, Any]:
        payload = event.payload
        yaml_path = payload.get('yaml_path')
        image_path = payload.get('image_path')
        if yaml_path and image_path:
            yaml_file = Path(str(yaml_path)).expanduser()
            image_file = Path(str(image_path)).expanduser()
            for path in (yaml_file, image_file):
                if not path.is_file():
                    raise MissionUploadError(
                        f'Map artifact is missing: {path}',
                        retryable=False,
                    )
            response = self._multipart_request(
                'POST',
                self.map_upload_path,
                {},
                [
                    (self.map_yaml_field, yaml_file),
                    (self.map_image_field, image_file),
                ],
                event.event_key,
            )
            server_map_id = response.get('map_id')
            if server_map_id and server_map_id != payload.get('map_id'):
                raise MissionUploadError(
                    'Server map_id does not match the Jetson revision: '
                    f'{server_map_id} != {payload.get("map_id")}',
                    retryable=False,
                )
            if not server_map_id:
                raise MissionUploadError(
                    'Map upload ACK has no map_id',
                    retryable=False,
                )
            return response

        raise MissionUploadError(
            'Server map upload contract supports YAML/PGM maps only; '
            'a Slam Toolbox posegraph endpoint has not been defined',
            retryable=False,
        )

    def _upload_detection_image(
        self,
        event: OutboxEvent,
    ) -> dict[str, Any]:
        path = Path(event.file_path).expanduser()
        if not path.is_file():
            raise MissionUploadError(
                f'Detection image is missing: {path}',
                retryable=False,
            )
        detection_id = quote(
            str(event.payload['detection_id']),
            safe='',
        )
        checksum = str(event.payload.get('image_checksum', '')).strip()
        if not checksum:
            checksum = hashlib.sha256(path.read_bytes()).hexdigest()
        response = self._multipart_request(
            'POST',
            f'/api/v1/detections/{detection_id}/images',
            {'checksum': checksum},
            [('file', path)],
            event.event_key,
        )
        if not response.get('image_id'):
            raise MissionUploadError(
                'Detection image ACK has no image_id',
                retryable=False,
            )
        if response.get('checksum') != checksum:
            raise MissionUploadError(
                'Detection image ACK checksum does not match request: '
                f'{response.get("checksum")} != {checksum}',
                retryable=False,
            )
        return response

    @staticmethod
    def _require_ack_identifier(
        response: dict[str, Any],
        field: str,
        expected: str,
    ) -> None:
        actual = response.get(field)
        if actual != expected:
            raise MissionUploadError(
                f'Server ACK {field} does not match request: '
                f'{actual!r} != {expected!r}',
                retryable=False,
            )

    def _json_request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any],
        idempotency_key: str,
    ) -> dict[str, Any]:
        data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(',', ':'),
        ).encode()
        return self._request(
            method,
            path,
            data,
            'application/json',
            idempotency_key,
        )

    def _multipart_request(
        self,
        method: str,
        path: str,
        fields: dict[str, str],
        files: list[tuple[str, Path]],
        idempotency_key: str,
    ) -> dict[str, Any]:
        data, content_type = encode_multipart(fields, files)
        return self._request(
            method,
            path,
            data,
            content_type,
            idempotency_key,
        )

    def _request(
        self,
        method: str,
        path: str,
        data: bytes | None,
        content_type: str,
        idempotency_key: str,
    ) -> dict[str, Any]:
        headers = {
            'Accept': 'application/json',
            'User-Agent': 'h753-jetson-mission-uploader/1',
        }
        if content_type:
            headers['Content-Type'] = content_type
        if idempotency_key:
            headers['Idempotency-Key'] = idempotency_key
        if self.token:
            headers[self.api_key_header] = self.token
        request = Request(
            f'{self.base_url}{path}',
            data=data,
            headers=headers,
            method=method,
        )
        try:
            response_context = self.opener(
                request,
                timeout=self.timeout_s,
            )
            with response_context as response:
                status = int(response.getcode())
                body = response.read()
        except HTTPError as exc:
            detail = exc.read(500).decode(errors='replace')
            raise MissionUploadError(
                f'HTTP {exc.code} for {path}: {detail}',
                retryable=(
                    exc.code in (408, 425, 429)
                    or 500 <= exc.code < 600
                ),
                status_code=int(exc.code),
            ) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise MissionUploadError(
                f'Upload failed for {path}: {exc}',
                retryable=True,
            ) from exc
        if not 200 <= status < 300:
            raise MissionUploadError(
                f'HTTP {status} for {path}',
                retryable=(
                    status in (408, 425, 429)
                    or 500 <= status < 600
                ),
                status_code=status,
            )
        if not body:
            return {'status': status}
        try:
            decoded = json.loads(body.decode())
        except (UnicodeDecodeError, ValueError):
            return {'status': status}
        return decoded if isinstance(decoded, dict) else {'data': decoded}
