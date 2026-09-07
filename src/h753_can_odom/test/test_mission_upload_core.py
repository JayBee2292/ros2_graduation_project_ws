import hashlib
import io
import json
from pathlib import Path
from urllib.error import HTTPError

import pytest

from h753_can_odom.mission_data_core import OutboxEvent
from h753_can_odom.mission_upload_core import (
    MissionApiClient,
    MissionUploadError,
    encode_multipart,
)


class FakeResponse:
    def __init__(self, status=201, payload=b'{"ack":true}'):
        self.status = status
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def getcode(self):
        return self.status

    def read(self):
        return self.payload


def test_api_client_rejects_plain_http_by_default() -> None:
    with pytest.raises(ValueError, match='Plain HTTP'):
        MissionApiClient('http://192.0.2.10:8000')


def test_route_batch_uses_idempotency_key_and_expected_endpoint() -> None:
    captured = {}

    def opener(request, timeout):
        captured['request'] = request
        captured['timeout'] = timeout
        return FakeResponse(payload=b'{"mission_id":"mission 1"}')

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        token='test-token',
        timeout_s=3.0,
        allow_insecure_http=True,
        opener=opener,
    )
    response = client.upload_route_batch(
        'mission 1',
        [
            {
                'route_seq': 4,
                'segment_id': 0,
                'stamp_ns': 10,
                'x': 1.0,
                'y': 2.0,
                'yaw': 0.3,
                'quality_flags': '',
            }
        ],
    )

    request = captured['request']
    assert response == {'mission_id': 'mission 1'}
    assert request.full_url.endswith(
        '/api/v1/missions/mission%201/route-points:batch'
    )
    assert request.headers['Idempotency-key'] == (
        'route:mission 1:4-4'
    )
    assert request.headers['X-api-key'] == 'test-token'
    assert captured['timeout'] == 3.0
    payload = json.loads(request.data.decode())
    assert 'mission_id' not in payload
    assert 'segment_id' not in payload['points'][0]
    assert payload['points'][0]['route_seq'] == 4


def test_detection_image_upload_is_multipart(tmp_path) -> None:
    captured = {}
    image = tmp_path / 'person.jpg'
    image.write_bytes(b'person-jpeg')
    checksum = hashlib.sha256(image.read_bytes()).hexdigest()

    def opener(request, timeout):
        captured['request'] = request
        return FakeResponse(
            payload=json.dumps(
                {'image_id': 'server-image-1', 'checksum': checksum}
            ).encode()
        )

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        allow_insecure_http=True,
        opener=opener,
    )
    event = OutboxEvent(
        event_key='detection:d1:image:i1',
        event_type='detection_image',
        mission_id='m1',
        payload={
            'detection_id': 'd1',
            'image_id': 'i1',
            'image_checksum': checksum,
        },
        file_path=str(image),
        attempts=0,
    )
    client.upload_event(event)

    request = captured['request']
    assert request.full_url.endswith('/api/v1/detections/d1/images')
    assert request.headers['Content-type'].startswith('multipart/form-data')
    assert b'person-jpeg' in request.data
    assert b'name="file"; filename="person.jpg"' in request.data
    assert b'name="checksum"' in request.data
    assert checksum.encode() in request.data
    assert b'name="metadata"' not in request.data


def test_encode_multipart_includes_named_file_and_metadata(tmp_path) -> None:
    artifact = tmp_path / 'map.yaml'
    artifact.write_text('image: map.pgm', encoding='utf-8')

    body, content_type = encode_multipart(
        {'metadata': '{"map_id":"one"}'},
        [('yaml_file', Path(artifact))],
    )

    assert content_type.startswith('multipart/form-data; boundary=')
    assert b'name="metadata"' in body
    assert b'name="yaml_file"; filename="map.yaml"' in body
    assert b'image: map.pgm' in body


def test_map_upload_uses_server_endpoint_and_map_id(tmp_path) -> None:
    captured = {}
    yaml_file = tmp_path / 'map.yaml'
    pgm_file = tmp_path / 'map.pgm'
    yaml_file.write_text('image: map.pgm', encoding='utf-8')
    pgm_file.write_bytes(b'P5\n1 1\n255\n\x00')

    def opener(request, timeout):
        captured['request'] = request
        return FakeResponse(payload=b'{"map_id":"map-expected"}')

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        allow_insecure_http=True,
        opener=opener,
    )
    event = OutboxEvent(
        event_key='map:map-expected',
        event_type='map',
        mission_id='mission-1',
        payload={
            'map_id': 'map-expected',
            'yaml_path': str(yaml_file),
            'image_path': str(pgm_file),
        },
        file_path='',
        attempts=0,
    )

    client.upload_event(event)

    request = captured['request']
    assert request.full_url.endswith('/api/v1/maps:upload')
    assert b'name="yaml_file"; filename="map.yaml"' in request.data
    assert b'name="pgm_file"; filename="map.pgm"' in request.data
    assert b'name="metadata"' not in request.data


def test_map_upload_rejects_server_revision_mismatch(tmp_path) -> None:
    yaml_file = tmp_path / 'map.yaml'
    pgm_file = tmp_path / 'map.pgm'
    yaml_file.write_text('image: map.pgm', encoding='utf-8')
    pgm_file.write_bytes(b'P5\n1 1\n255\n\x00')

    def opener(request, timeout):
        return FakeResponse(payload=b'{"map_id":"different-map"}')

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        allow_insecure_http=True,
        opener=opener,
    )
    event = OutboxEvent(
        event_key='map:expected-map',
        event_type='map',
        mission_id='mission-1',
        payload={
            'map_id': 'expected-map',
            'yaml_path': str(yaml_file),
            'image_path': str(pgm_file),
        },
        file_path='',
        attempts=0,
    )

    with pytest.raises(MissionUploadError, match='does not match') as error:
        client.upload_event(event)
    assert error.value.retryable is False


def test_health_verifies_server_schema_version() -> None:
    captured = {}

    def opener(request, timeout):
        captured['request'] = request
        return FakeResponse(
            status=200,
            payload=b'{"status":"ok","schema_version":1}',
        )

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        allow_insecure_http=True,
        opener=opener,
    )

    assert client.health()['status'] == 'ok'
    assert captured['request'].method == 'GET'
    assert captured['request'].data is None
    assert captured['request'].full_url.endswith('/api/v1/health')


def test_mission_and_detection_payloads_match_server_models() -> None:
    requests = []

    def opener(request, timeout):
        requests.append(request)
        payload = json.loads(request.data.decode())
        if request.full_url.endswith('/api/v1/missions'):
            ack = {'mission_id': payload['mission_id']}
        elif request.method == 'PATCH':
            ack = {'mission_id': 'm1', 'status': payload['status']}
        else:
            ack = {'detection_id': payload['detection_id']}
        return FakeResponse(payload=json.dumps(ack).encode())

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        allow_insecure_http=True,
        opener=opener,
    )
    client.upload_event(
        OutboxEvent(
            event_key='mission:m1:start',
            event_type='mission_start',
            mission_id='m1',
            payload={
                'schema_version': 1,
                'mission_id': 'm1',
                'robot_id': 'r1',
                'map_id': 'map1',
                'started_at': '2026-09-07T00:00:00+00:00',
                'started_at_ns': 1,
                'status': 'ACTIVE',
            },
            file_path='',
            attempts=0,
        )
    )
    client.upload_event(
        OutboxEvent(
            event_key='detection:d1',
            event_type='detection',
            mission_id='m1',
            payload={
                'schema_version': 1,
                'mission_id': 'm1',
                'detection_id': 'd1',
                'detected_at': '2026-09-07T00:00:01+00:00',
                'detected_at_ns': 2,
                'map_id': 'map1',
                'frame_id': 'map',
                'robot_pose': {
                    'x': 1.0,
                    'y': 2.0,
                    'yaw': 0.5,
                    'stamp_ns': 2,
                },
                'route_end_seq': 4,
                'yolo': {'person_found': True, 'unused': 'drop'},
                'image_id': 'local-image',
            },
            file_path='',
            attempts=0,
        )
    )
    client.upload_event(
        OutboxEvent(
            event_key='mission:m1:end',
            event_type='mission_end',
            mission_id='m1',
            payload={
                'schema_version': 1,
                'mission_id': 'm1',
                'status': 'ABORTED',
                'ended_at': '2026-09-07T00:00:02+00:00',
                'ended_at_ns': 3,
                'end_reason': 'MAP_CHANGED',
            },
            file_path='',
            attempts=0,
        )
    )

    start = json.loads(requests[0].data.decode())
    detection = json.loads(requests[1].data.decode())
    end = json.loads(requests[2].data.decode())
    assert set(start) == {
        'schema_version', 'mission_id', 'robot_id', 'map_id', 'started_at'
    }
    assert set(detection) == {
        'schema_version', 'mission_id', 'detection_id', 'detected_at',
        'map_id', 'frame_id', 'robot_pose', 'route_end_seq', 'yolo'
    }
    assert set(detection['robot_pose']) == {'x', 'y', 'yaw'}
    assert detection['yolo'] == {'person_found': True}
    assert end == {
        'status': 'ABORTED',
        'ended_at': '2026-09-07T00:00:02+00:00',
    }


@pytest.mark.parametrize(
    ('status', 'retryable'),
    [(401, False), (409, False), (429, True), (500, True)],
)
def test_http_errors_are_classified(status, retryable) -> None:
    def opener(request, timeout):
        raise HTTPError(
            request.full_url,
            status,
            'failed',
            {},
            io.BytesIO(b'{"detail":"failed"}'),
        )

    client = MissionApiClient(
        'http://127.0.0.1:8000',
        allow_insecure_http=True,
        opener=opener,
    )
    event = OutboxEvent(
        event_key='mission:m1:start',
        event_type='mission_start',
        mission_id='m1',
        payload={
            'schema_version': 1,
            'mission_id': 'm1',
            'robot_id': 'r1',
            'map_id': 'map1',
            'started_at': '2026-09-07T00:00:00+00:00',
        },
        file_path='',
        attempts=0,
    )

    with pytest.raises(MissionUploadError) as error:
        client.upload_event(event)
    assert error.value.status_code == status
    assert error.value.retryable is retryable
