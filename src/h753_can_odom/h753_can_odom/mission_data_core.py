from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sqlite3
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable
from uuid import uuid4

import yaml


SCHEMA_VERSION = 1
VALID_MISSION_STATUSES = ('ACTIVE', 'COMPLETED', 'ABORTED')
OUTBOX_PRIORITIES = {
    'map': 10,
    'mission_start': 20,
    'detection': 40,
    'detection_image': 50,
    'mission_end': 60,
}

NAVIGATION_GOAL_ACTIVE_STATUSES = frozenset((1, 2, 3))
NAVIGATION_GOAL_TERMINAL_EVENTS = {
    4: 'succeeded',
    5: 'canceled',
    6: 'aborted',
}


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(first: float, second: float) -> float:
    return abs(normalize_angle(second - first))


def stamp_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


def ns_to_iso8601(stamp_ns: int) -> str:
    return datetime.fromtimestamp(
        stamp_ns / 1_000_000_000,
        tz=timezone.utc,
    ).isoformat(timespec='milliseconds')


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    x: float
    y: float
    yaw: float

    def distance_from(self, other: 'PoseSample') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class RouteDecision:
    record: bool
    pose_jump: bool = False


@dataclass(frozen=True)
class NavigationGoalTransition:
    event: str
    goal_id: str
    previous_goal_id: str | None = None


@dataclass(frozen=True)
class NavigationMissionTransition:
    event: str
    source: str
    goal_id: str
    previous_source: str | None = None
    previous_goal_id: str | None = None


class NavigationGoalTracker:
    """Reduce repeated Nav2 action status arrays to one lifecycle event."""

    def __init__(self) -> None:
        self.active_goal_id: str | None = None
        self._terminal_goal_ids: set[str] = set()

    def reset(self) -> None:
        self.active_goal_id = None

    def observe(
        self,
        goal_id: str,
        status: int,
    ) -> NavigationGoalTransition | None:
        if not goal_id or goal_id in self._terminal_goal_ids:
            return None

        if status in NAVIGATION_GOAL_ACTIVE_STATUSES:
            if self.active_goal_id is None:
                self.active_goal_id = goal_id
                return NavigationGoalTransition('started', goal_id)
            if self.active_goal_id == goal_id:
                return None
            previous = self.active_goal_id
            self.active_goal_id = goal_id
            return NavigationGoalTransition('replaced', goal_id, previous)

        event = NAVIGATION_GOAL_TERMINAL_EVENTS.get(status)
        if event is None or self.active_goal_id != goal_id:
            return None
        self.active_goal_id = None
        self._terminal_goal_ids.add(goal_id)
        return NavigationGoalTransition(event, goal_id)


class NavigationMissionTracker:
    """Track one user-visible route across nested Nav2 actions.

    ``FollowWaypoints`` owns a sequence of internal ``NavigateToPose`` goals.
    Those child results must not close the mission.  Parent route actions take
    precedence while preserving standalone NavigateToPose behavior.
    """

    def __init__(
        self,
        parent_sources: Iterable[str] = (
            'follow_waypoints',
            'navigate_through_poses',
        ),
    ) -> None:
        self.parent_sources = frozenset(parent_sources)
        self._trackers: dict[str, NavigationGoalTracker] = {}
        self.active_source: str | None = None

    @property
    def active_goal_id(self) -> str | None:
        if self.active_source is None:
            return None
        return self._tracker(self.active_source).active_goal_id

    def _tracker(self, source: str) -> NavigationGoalTracker:
        tracker = self._trackers.get(source)
        if tracker is None:
            tracker = NavigationGoalTracker()
            self._trackers[source] = tracker
        return tracker

    def reset(self) -> None:
        self.active_source = None
        for tracker in self._trackers.values():
            tracker.reset()

    def observe(
        self,
        source: str,
        goal_id: str,
        status: int,
    ) -> NavigationMissionTransition | None:
        transition = self._tracker(source).observe(goal_id, status)
        if transition is None:
            return None

        active_is_parent = self.active_source in self.parent_sources
        source_is_parent = source in self.parent_sources

        # FollowWaypoints emits one NavigateToPose lifecycle per waypoint.
        # Consume those child statuses for deduplication, but never expose them
        # as mission boundaries while the parent route is active.
        if active_is_parent and not source_is_parent:
            return None

        if transition.event in ('started', 'replaced'):
            previous_source = self.active_source
            previous_goal_id = self.active_goal_id

            if source_is_parent and previous_source not in (None, source):
                self._tracker(previous_source).reset()
                self.active_source = source
                event = (
                    'promoted'
                    if previous_source not in self.parent_sources
                    else 'replaced'
                )
                return NavigationMissionTransition(
                    event,
                    source,
                    goal_id,
                    previous_source,
                    previous_goal_id,
                )

            if previous_source is None:
                self.active_source = source
                return NavigationMissionTransition('started', source, goal_id)

            if previous_source == source:
                self.active_source = source
                return NavigationMissionTransition(
                    transition.event,
                    source,
                    goal_id,
                    source,
                    transition.previous_goal_id,
                )

            return None

        if self.active_source != source:
            return None

        self.active_source = None
        return NavigationMissionTransition(
            transition.event,
            source,
            goal_id,
        )


def decide_route_sample(
    previous: PoseSample | None,
    candidate: PoseSample,
    elapsed_since_record_s: float,
    min_distance_m: float,
    min_yaw_rad: float,
    max_interval_s: float,
    max_pose_jump_m: float,
    max_yaw_jump_rad: float,
    force: bool = False,
) -> RouteDecision:
    if previous is None:
        return RouteDecision(True)

    distance = candidate.distance_from(previous)
    yaw_delta = angular_distance(previous.yaw, candidate.yaw)
    pose_jump = (
        distance > max_pose_jump_m
        or yaw_delta > max_yaw_jump_rad
    )
    record = (
        force
        or pose_jump
        or distance >= min_distance_m
        or yaw_delta >= min_yaw_rad
        or elapsed_since_record_s >= max_interval_s
    )
    return RouteDecision(record, pose_jump)


def _read_pgm_size(path: Path) -> tuple[int, int]:
    tokens: list[bytes] = []
    with path.open('rb') as stream:
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                break
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 4 or tokens[0] not in (b'P2', b'P5'):
        raise ValueError(f'Unsupported or malformed PGM: {path}')
    return int(tokens[1]), int(tokens[2])


def _sha256_file(path: Path, digest: Any | None = None) -> str:
    file_digest = hashlib.sha256() if digest is None else digest
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            file_digest.update(chunk)
    return file_digest.hexdigest()


@dataclass(frozen=True)
class MapManifest:
    map_id: str
    backend: str
    frame_id: str
    yaml_path: str = ''
    image_path: str = ''
    posegraph_path: str = ''
    posegraph_data_path: str = ''
    resolution: float | None = None
    origin_x: float | None = None
    origin_y: float | None = None
    origin_yaw: float | None = None
    width: int | None = None
    height: int | None = None
    yaml_checksum: str = ''
    image_checksum: str = ''
    posegraph_checksum: str = ''
    posegraph_data_checksum: str = ''

    @classmethod
    def from_amcl_yaml(
        cls,
        yaml_path: str | Path,
        frame_id: str = 'map',
    ) -> 'MapManifest':
        yaml_file = Path(yaml_path).expanduser().resolve()
        with yaml_file.open(encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream) or {}
        image_value = str(metadata.get('image', '')).strip()
        if not image_value:
            raise ValueError(f'Map YAML has no image field: {yaml_file}')
        image_file = Path(image_value).expanduser()
        if not image_file.is_absolute():
            image_file = yaml_file.parent / image_file
        image_file = image_file.resolve()

        origin = metadata.get('origin')
        if not isinstance(origin, list) or len(origin) < 3:
            raise ValueError(f'Map YAML has invalid origin: {yaml_file}')
        resolution = float(metadata['resolution'])
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError(f'Map YAML has invalid resolution: {yaml_file}')

        # Keep this byte-for-byte compatible with the server map upload API.
        # The server computes SHA-256 over YAML bytes followed immediately by
        # PGM bytes and exposes the first 16 hex characters as ``map_<hash>``.
        digest = hashlib.sha256()
        digest.update(yaml_file.read_bytes())
        _sha256_file(image_file, digest)
        map_id = f'map_{digest.hexdigest()[:16]}'
        width, height = _read_pgm_size(image_file)
        return cls(
            map_id=map_id,
            backend='amcl',
            frame_id=frame_id,
            yaml_path=str(yaml_file),
            image_path=str(image_file),
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            width=width,
            height=height,
            yaml_checksum=_sha256_file(yaml_file),
            image_checksum=_sha256_file(image_file),
        )

    @classmethod
    def from_posegraph(
        cls,
        base_path: str | Path,
        frame_id: str = 'map',
    ) -> 'MapManifest':
        base = Path(base_path).expanduser().resolve()
        posegraph = Path(f'{base}.posegraph')
        data = Path(f'{base}.data')
        if not posegraph.is_file() or not data.is_file():
            raise FileNotFoundError(
                f'Slam Toolbox map files are missing: {posegraph}, {data}'
            )
        posegraph_checksum = _sha256_file(posegraph)
        data_checksum = _sha256_file(data)
        digest = hashlib.sha256()
        digest.update(b'slam-toolbox-map-v1\0')
        digest.update(posegraph_checksum.encode())
        digest.update(b'\0')
        digest.update(data_checksum.encode())
        return cls(
            map_id=digest.hexdigest(),
            backend='slam_toolbox',
            frame_id=frame_id,
            posegraph_path=str(posegraph),
            posegraph_data_path=str(data),
            posegraph_checksum=posegraph_checksum,
            posegraph_data_checksum=data_checksum,
        )

    @classmethod
    def from_sources(
        cls,
        backend: str,
        map_yaml_path: str | Path,
        posegraph_base_path: str | Path,
        frame_id: str = 'map',
    ) -> 'MapManifest':
        normalized = backend.strip().lower()
        if normalized == 'amcl':
            return cls.from_amcl_yaml(map_yaml_path, frame_id)
        if normalized == 'slam_toolbox':
            return cls.from_posegraph(posegraph_base_path, frame_id)
        raise ValueError(f'Unsupported localization backend: {backend}')

    def as_dict(self) -> dict[str, Any]:
        return {
            'schema_version': SCHEMA_VERSION,
            'map_id': self.map_id,
            'backend': self.backend,
            'frame_id': self.frame_id,
            'yaml_path': self.yaml_path,
            'image_path': self.image_path,
            'posegraph_path': self.posegraph_path,
            'posegraph_data_path': self.posegraph_data_path,
            'resolution': self.resolution,
            'origin': (
                None
                if self.origin_x is None
                else [self.origin_x, self.origin_y, self.origin_yaw]
            ),
            'width': self.width,
            'height': self.height,
            'checksums': {
                'yaml': self.yaml_checksum,
                'image': self.image_checksum,
                'posegraph': self.posegraph_checksum,
                'posegraph_data': self.posegraph_data_checksum,
            },
        }


class DetectionEventGate:
    """Create one event per gateway-authorized YOLO detection cycle."""

    def __init__(
        self,
        gate_topics: Iterable[str],
        fallback_cooldown_s: float = 15.0,
        fallback_clear_s: float = 2.0,
    ) -> None:
        topics = tuple(gate_topics)
        if not topics:
            raise ValueError('gate_topics must not be empty')
        self.gate_state = {topic: False for topic in topics}
        self.fallback_cooldown_s = fallback_cooldown_s
        self.fallback_clear_s = fallback_clear_s
        self.local_armed = True
        self.gateway_armed: bool | None = None
        self.disarmed_at: float | None = None
        self.clear_since: float | None = None

    @property
    def active(self) -> bool:
        return any(self.gate_state.values())

    def update_gateway(self, armed: bool, now: float) -> bool:
        self.gateway_armed = armed
        if armed and not self.active and not self.local_armed:
            self.local_armed = True
            self.disarmed_at = None
            self.clear_since = None
            return True
        if not armed:
            self.local_armed = False
            if self.disarmed_at is None:
                self.disarmed_at = now
        return False

    def update_gate(self, topic: str, value: int, now: float) -> bool:
        previous = self.gate_state.get(topic, False)
        active = value != 0
        self.gate_state[topic] = active
        self._update_clear(now)
        rising = active and not previous
        if not rising or not self.local_armed:
            return False
        if self.gateway_armed is False:
            return False
        self.local_armed = False
        self.disarmed_at = now
        self.clear_since = None
        return True

    def tick(self, now: float) -> bool:
        if self.local_armed:
            return False
        self._update_clear(now)
        if self.gateway_armed is not None:
            return False
        if self.disarmed_at is None or self.clear_since is None:
            return False
        if now - self.disarmed_at < self.fallback_cooldown_s:
            return False
        if now - self.clear_since < self.fallback_clear_s:
            return False
        self.local_armed = True
        self.disarmed_at = None
        self.clear_since = None
        return True

    def _update_clear(self, now: float) -> None:
        if self.local_armed:
            return
        if self.active:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = now


@dataclass(frozen=True)
class OutboxEvent:
    event_key: str
    event_type: str
    mission_id: str
    payload: dict[str, Any]
    file_path: str
    attempts: int


class MissionStore:
    """Persistent Jetson mission record and upload outbox."""

    def __init__(self, database_path: str | Path) -> None:
        self.database_path = Path(database_path).expanduser()
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(
            str(self.database_path),
            timeout=5.0,
            check_same_thread=False,
        )
        self._connection.row_factory = sqlite3.Row
        self._connection.execute('PRAGMA journal_mode=WAL')
        self._connection.execute('PRAGMA foreign_keys=ON')
        self._connection.execute('PRAGMA busy_timeout=5000')
        self._initialize_schema()

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def _initialize_schema(self) -> None:
        with self._lock, self._connection:
            self._connection.executescript(
                '''
                CREATE TABLE IF NOT EXISTS schema_meta (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS missions (
                    mission_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    map_id TEXT NOT NULL,
                    map_manifest_json TEXT NOT NULL,
                    started_at_ns INTEGER NOT NULL,
                    ended_at_ns INTEGER,
                    end_reason TEXT NOT NULL DEFAULT '',
                    start_mode INTEGER,
                    status TEXT NOT NULL,
                    created_at_ns INTEGER NOT NULL
                );

                CREATE TABLE IF NOT EXISTS route_points (
                    mission_id TEXT NOT NULL,
                    route_seq INTEGER NOT NULL,
                    segment_id INTEGER NOT NULL,
                    stamp_ns INTEGER NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    yaw REAL NOT NULL,
                    quality_flags TEXT NOT NULL DEFAULT '',
                    upload_state TEXT NOT NULL DEFAULT 'PENDING',
                    upload_attempts INTEGER NOT NULL DEFAULT 0,
                    upload_error TEXT NOT NULL DEFAULT '',
                    PRIMARY KEY (mission_id, route_seq),
                    FOREIGN KEY (mission_id) REFERENCES missions(mission_id)
                );

                CREATE INDEX IF NOT EXISTS idx_route_stamp
                ON route_points(mission_id, stamp_ns);

                CREATE TABLE IF NOT EXISTS detections (
                    detection_id TEXT PRIMARY KEY,
                    mission_id TEXT NOT NULL,
                    detected_at_ns INTEGER NOT NULL,
                    pose_stamp_ns INTEGER NOT NULL,
                    route_end_seq INTEGER NOT NULL,
                    map_id TEXT NOT NULL,
                    frame_id TEXT NOT NULL,
                    robot_x REAL NOT NULL,
                    robot_y REAL NOT NULL,
                    robot_yaw REAL NOT NULL,
                    payload_json TEXT NOT NULL,
                    image_id TEXT,
                    image_path TEXT,
                    image_stamp_ns INTEGER,
                    FOREIGN KEY (mission_id) REFERENCES missions(mission_id)
                );

                CREATE TABLE IF NOT EXISTS outbox (
                    event_key TEXT PRIMARY KEY,
                    event_type TEXT NOT NULL,
                    mission_id TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    file_path TEXT NOT NULL DEFAULT '',
                    state TEXT NOT NULL DEFAULT 'PENDING',
                    attempts INTEGER NOT NULL DEFAULT 0,
                    next_attempt_ns INTEGER NOT NULL DEFAULT 0,
                    last_error TEXT NOT NULL DEFAULT '',
                    ack_json TEXT NOT NULL DEFAULT '',
                    created_at_ns INTEGER NOT NULL,
                    sent_at_ns INTEGER
                );

                CREATE INDEX IF NOT EXISTS idx_outbox_pending
                ON outbox(state, next_attempt_ns, created_at_ns);
                '''
            )
            self._connection.execute(
                '''
                INSERT INTO schema_meta(key, value) VALUES('schema_version', ?)
                ON CONFLICT(key) DO UPDATE SET value=excluded.value
                ''',
                (str(SCHEMA_VERSION),),
            )
            detection_columns = {
                row['name']
                for row in self._connection.execute(
                    'PRAGMA table_info(detections)'
                ).fetchall()
            }
            if 'pose_stamp_ns' not in detection_columns:
                self._connection.execute(
                    'ALTER TABLE detections '
                    'ADD COLUMN pose_stamp_ns INTEGER NOT NULL DEFAULT 0'
                )
            mission_columns = {
                row['name']
                for row in self._connection.execute(
                    'PRAGMA table_info(missions)'
                ).fetchall()
            }
            if 'end_reason' not in mission_columns:
                self._connection.execute(
                    "ALTER TABLE missions "
                    "ADD COLUMN end_reason TEXT NOT NULL DEFAULT ''"
                )
            route_columns = {
                row['name']
                for row in self._connection.execute(
                    'PRAGMA table_info(route_points)'
                ).fetchall()
            }
            if 'upload_attempts' not in route_columns:
                self._connection.execute(
                    'ALTER TABLE route_points '
                    'ADD COLUMN upload_attempts INTEGER NOT NULL DEFAULT 0'
                )
            if 'upload_error' not in route_columns:
                self._connection.execute(
                    "ALTER TABLE route_points "
                    "ADD COLUMN upload_error TEXT NOT NULL DEFAULT ''"
                )
            outbox_columns = {
                row['name']
                for row in self._connection.execute(
                    'PRAGMA table_info(outbox)'
                ).fetchall()
            }
            if 'ack_json' not in outbox_columns:
                self._connection.execute(
                    "ALTER TABLE outbox "
                    "ADD COLUMN ack_json TEXT NOT NULL DEFAULT ''"
                )

    @staticmethod
    def _json(payload: dict[str, Any]) -> str:
        return json.dumps(
            payload,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )

    def _queue_event(
        self,
        connection: sqlite3.Connection,
        event_key: str,
        event_type: str,
        mission_id: str,
        payload: dict[str, Any],
        created_at_ns: int,
        file_path: str = '',
    ) -> None:
        connection.execute(
            '''
            INSERT INTO outbox(
                event_key, event_type, mission_id, payload_json,
                file_path, created_at_ns
            ) VALUES(?, ?, ?, ?, ?, ?)
            ON CONFLICT(event_key) DO NOTHING
            ''',
            (
                event_key,
                event_type,
                mission_id,
                self._json(payload),
                file_path,
                created_at_ns,
            ),
        )

    def active_mission(self) -> dict[str, Any] | None:
        with self._lock:
            row = self._connection.execute(
                '''
                SELECT * FROM missions
                WHERE status='ACTIVE'
                ORDER BY started_at_ns DESC LIMIT 1
                '''
            ).fetchone()
        return None if row is None else dict(row)

    def create_or_resume_mission(
        self,
        robot_id: str,
        manifest: MapManifest,
        stamp_ns: int,
        start_mode: int | None,
        force_new: bool = False,
    ) -> tuple[dict[str, Any], bool]:
        with self._lock, self._connection:
            active = self._connection.execute(
                '''
                SELECT * FROM missions
                WHERE status='ACTIVE'
                ORDER BY started_at_ns DESC LIMIT 1
                '''
            ).fetchone()
            if active is not None and (
                force_new
                or active['robot_id'] != robot_id
                or active['map_id'] != manifest.map_id
            ):
                if force_new:
                    end_status = 'COMPLETED'
                    end_reason = 'NEW_MISSION_REQUESTED'
                elif active['robot_id'] != robot_id:
                    end_status = 'ABORTED'
                    end_reason = 'ROBOT_ID_CHANGED'
                else:
                    end_status = 'ABORTED'
                    end_reason = 'MAP_CHANGED'
                self._end_mission_locked(
                    self._connection,
                    active['mission_id'],
                    stamp_ns,
                    end_status,
                    end_reason,
                )
                active = None
            if active is not None:
                return dict(active), False

            mission_id = str(uuid4())
            map_payload = manifest.as_dict()
            mission_payload = {
                'schema_version': SCHEMA_VERSION,
                'mission_id': mission_id,
                'robot_id': robot_id,
                'map_id': manifest.map_id,
                'started_at': ns_to_iso8601(stamp_ns),
                'started_at_ns': stamp_ns,
                'start_mode': start_mode,
                'status': 'ACTIVE',
            }
            self._connection.execute(
                '''
                INSERT INTO missions(
                    mission_id, robot_id, map_id, map_manifest_json,
                    started_at_ns, start_mode, status, created_at_ns
                ) VALUES(?, ?, ?, ?, ?, ?, 'ACTIVE', ?)
                ''',
                (
                    mission_id,
                    robot_id,
                    manifest.map_id,
                    self._json(map_payload),
                    stamp_ns,
                    start_mode,
                    time.time_ns(),
                ),
            )
            self._queue_event(
                self._connection,
                f'map:{manifest.map_id}',
                'map',
                mission_id,
                map_payload,
                stamp_ns,
            )
            self._queue_event(
                self._connection,
                f'mission:{mission_id}:start',
                'mission_start',
                mission_id,
                mission_payload,
                stamp_ns,
            )
            row = self._connection.execute(
                'SELECT * FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            return dict(row), True

    def end_active_mission(
        self,
        stamp_ns: int,
        status: str = 'COMPLETED',
        end_reason: str = 'OPERATOR_REQUESTED',
    ) -> str | None:
        if status not in VALID_MISSION_STATUSES[1:]:
            raise ValueError(
                'Mission end status must be COMPLETED or ABORTED'
            )
        with self._lock, self._connection:
            row = self._connection.execute(
                '''
                SELECT mission_id FROM missions
                WHERE status='ACTIVE'
                ORDER BY started_at_ns DESC LIMIT 1
                '''
            ).fetchone()
            if row is None:
                return None
            self._end_mission_locked(
                self._connection,
                row['mission_id'],
                stamp_ns,
                status,
                end_reason,
            )
            return str(row['mission_id'])

    def _end_mission_locked(
        self,
        connection: sqlite3.Connection,
        mission_id: str,
        stamp_ns: int,
        status: str,
        end_reason: str,
    ) -> None:
        connection.execute(
            '''
            UPDATE missions SET status=?, ended_at_ns=?, end_reason=?
            WHERE mission_id=? AND status='ACTIVE'
            ''',
            (status, stamp_ns, end_reason, mission_id),
        )
        payload = {
            'schema_version': SCHEMA_VERSION,
            'mission_id': mission_id,
            'status': status,
            'ended_at': ns_to_iso8601(stamp_ns),
            'ended_at_ns': stamp_ns,
            'end_reason': end_reason,
        }
        self._queue_event(
            connection,
            f'mission:{mission_id}:end',
            'mission_end',
            mission_id,
            payload,
            stamp_ns,
        )

    def add_route_point(
        self,
        mission_id: str,
        sample: PoseSample,
        segment_id: int,
        quality_flags: str = '',
    ) -> int:
        with self._lock, self._connection:
            row = self._connection.execute(
                '''
                SELECT COALESCE(MAX(route_seq), -1) + 1 AS next_seq
                FROM route_points WHERE mission_id=?
                ''',
                (mission_id,),
            ).fetchone()
            route_seq = int(row['next_seq'])
            self._connection.execute(
                '''
                INSERT INTO route_points(
                    mission_id, route_seq, segment_id, stamp_ns,
                    x, y, yaw, quality_flags
                ) VALUES(?, ?, ?, ?, ?, ?, ?, ?)
                ''',
                (
                    mission_id,
                    route_seq,
                    segment_id,
                    sample.stamp_ns,
                    sample.x,
                    sample.y,
                    sample.yaw,
                    quality_flags,
                ),
            )
            return route_seq

    def latest_route_point(
        self,
        mission_id: str,
    ) -> dict[str, Any] | None:
        with self._lock:
            row = self._connection.execute(
                '''
                SELECT * FROM route_points WHERE mission_id=?
                ORDER BY route_seq DESC LIMIT 1
                ''',
                (mission_id,),
            ).fetchone()
        return None if row is None else dict(row)

    def route_points(
        self,
        mission_id: str,
        limit: int = 10000,
    ) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._connection.execute(
                '''
                SELECT * FROM route_points WHERE mission_id=?
                ORDER BY route_seq ASC LIMIT ?
                ''',
                (mission_id, limit),
            ).fetchall()
        return [dict(row) for row in rows]

    def add_detection(
        self,
        mission_id: str,
        manifest: MapManifest,
        sample: PoseSample,
        route_end_seq: int,
        yolo: dict[str, Any],
        image_id: str,
        detection_id: str | None = None,
        detected_at_ns: int | None = None,
    ) -> tuple[str, dict[str, Any]]:
        identifier = str(uuid4()) if detection_id is None else detection_id
        detection_stamp_ns = (
            sample.stamp_ns
            if detected_at_ns is None
            else int(detected_at_ns)
        )
        with self._lock:
            mission = self._connection.execute(
                'SELECT robot_id FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
        if mission is None:
            raise ValueError(f'Unknown mission_id: {mission_id}')
        payload = {
            'schema_version': SCHEMA_VERSION,
            'robot_id': str(mission['robot_id']),
            'mission_id': mission_id,
            'detection_id': identifier,
            'detected_at': ns_to_iso8601(detection_stamp_ns),
            'detected_at_ns': detection_stamp_ns,
            'map_id': manifest.map_id,
            'frame_id': manifest.frame_id,
            'robot_pose': {
                'x': sample.x,
                'y': sample.y,
                'yaw': sample.yaw,
                'stamp_ns': sample.stamp_ns,
            },
            'route_end_seq': route_end_seq,
            'yolo': yolo,
            'image_id': image_id,
        }
        with self._lock, self._connection:
            self._connection.execute(
                '''
                INSERT INTO detections(
                    detection_id, mission_id, detected_at_ns,
                    pose_stamp_ns, route_end_seq, map_id, frame_id,
                    robot_x, robot_y, robot_yaw,
                    payload_json, image_id
                ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''',
                (
                    identifier,
                    mission_id,
                    detection_stamp_ns,
                    sample.stamp_ns,
                    route_end_seq,
                    manifest.map_id,
                    manifest.frame_id,
                    sample.x,
                    sample.y,
                    sample.yaw,
                    self._json(payload),
                    image_id,
                ),
            )
            self._queue_event(
                self._connection,
                f'detection:{identifier}',
                'detection',
                mission_id,
                payload,
                detection_stamp_ns,
            )
        return identifier, payload

    def attach_detection_image(
        self,
        detection_id: str,
        image_id: str,
        image_path: str | Path,
        image_stamp_ns: int,
    ) -> bool:
        path = str(Path(image_path).expanduser())
        with self._lock, self._connection:
            detection = self._connection.execute(
                'SELECT * FROM detections WHERE detection_id=?',
                (detection_id,),
            ).fetchone()
            if detection is None:
                return False
            self._connection.execute(
                '''
                UPDATE detections
                SET image_id=?, image_path=?, image_stamp_ns=?
                WHERE detection_id=?
                ''',
                (image_id, path, image_stamp_ns, detection_id),
            )
            payload = {
                'schema_version': SCHEMA_VERSION,
                'mission_id': detection['mission_id'],
                'detection_id': detection_id,
                'image_id': image_id,
                'image_stamp_ns': image_stamp_ns,
                'image_checksum': _sha256_file(Path(path)),
            }
            self._queue_event(
                self._connection,
                f'detection:{detection_id}:image:{image_id}',
                'detection_image',
                detection['mission_id'],
                payload,
                time.time_ns(),
                file_path=path,
            )
            return True

    def next_outbox_event(self, now_ns: int) -> OutboxEvent | None:
        priority_sql = 'CASE event_type ' + ' '.join(
            f"WHEN '{event}' THEN {priority}"
            for event, priority in OUTBOX_PRIORITIES.items()
        ) + ' ELSE 100 END'
        with self._lock:
            row = self._connection.execute(
                f'''
                SELECT candidate.* FROM outbox AS candidate
                WHERE candidate.state IN ('PENDING', 'FAILED')
                  AND candidate.next_attempt_ns <= ?
                  AND (
                    candidate.event_type='map'
                    OR (
                      candidate.event_type='mission_start'
                      AND EXISTS(
                        SELECT 1
                        FROM missions AS mission
                        JOIN outbox AS dependency
                          ON dependency.event_key='map:' || mission.map_id
                        WHERE mission.mission_id=candidate.mission_id
                          AND dependency.state='SENT'
                      )
                    )
                    OR (
                      candidate.event_type='detection'
                      AND EXISTS(
                        SELECT 1 FROM outbox AS dependency
                        WHERE dependency.event_key=
                          'mission:' || candidate.mission_id || ':start'
                          AND dependency.state='SENT'
                      )
                    )
                    OR (
                      candidate.event_type='mission_end'
                      AND EXISTS(
                        SELECT 1 FROM outbox AS dependency
                        WHERE dependency.event_key=
                          'mission:' || candidate.mission_id || ':start'
                          AND dependency.state='SENT'
                      )
                      AND NOT EXISTS(
                        SELECT 1 FROM route_points AS route
                        WHERE route.mission_id=candidate.mission_id
                          AND route.upload_state!='SENT'
                      )
                    )
                    OR (
                      candidate.event_type='detection_image'
                      AND EXISTS(
                        SELECT 1 FROM outbox AS dependency
                        WHERE dependency.event_type='detection'
                          AND dependency.state='SENT'
                          AND candidate.event_key LIKE
                            dependency.event_key || ':image:%'
                      )
                    )
                  )
                ORDER BY {priority_sql}, created_at_ns ASC LIMIT 1
                ''',
                (now_ns,),
            ).fetchone()
        if row is None:
            return None
        return OutboxEvent(
            event_key=row['event_key'],
            event_type=row['event_type'],
            mission_id=row['mission_id'],
            payload=json.loads(row['payload_json']),
            file_path=row['file_path'],
            attempts=int(row['attempts']),
        )

    def mark_outbox_sent(
        self,
        event_key: str,
        sent_at_ns: int,
        acknowledgement: dict[str, Any] | None = None,
    ) -> None:
        ack_json = (
            '' if acknowledgement is None else self._json(acknowledgement)
        )
        with self._lock, self._connection:
            self._connection.execute(
                '''
                UPDATE outbox
                SET state='SENT', sent_at_ns=?, last_error='', ack_json=?
                WHERE event_key=?
                ''',
                (sent_at_ns, ack_json, event_key),
            )

    def mark_outbox_failed(
        self,
        event_key: str,
        error: str,
        next_attempt_ns: int,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                '''
                UPDATE outbox
                SET state='FAILED', attempts=attempts+1,
                    next_attempt_ns=?, last_error=?
                WHERE event_key=? AND state IN ('PENDING', 'FAILED')
                ''',
                (next_attempt_ns, error[:1000], event_key),
            )

    def mark_outbox_blocked(
        self,
        event_key: str,
        error: str,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                '''
                UPDATE outbox
                SET state='BLOCKED', attempts=attempts+1,
                    next_attempt_ns=0, last_error=?
                WHERE event_key=? AND state IN ('PENDING', 'FAILED')
                ''',
                (error[:1000], event_key),
            )

    def pending_route_batch(
        self,
        limit: int,
    ) -> tuple[str, list[dict[str, Any]]] | None:
        with self._lock:
            mission = self._connection.execute(
                '''
                SELECT m.mission_id
                FROM missions m
                JOIN outbox o
                  ON o.event_key='mission:' || m.mission_id || ':start'
                WHERE o.state='SENT'
                  AND EXISTS(
                    SELECT 1 FROM route_points r
                    WHERE r.mission_id=m.mission_id
                      AND r.upload_state='PENDING'
                  )
                ORDER BY m.started_at_ns ASC LIMIT 1
                '''
            ).fetchone()
            if mission is None:
                return None
            rows = self._connection.execute(
                '''
                SELECT route_seq, segment_id, stamp_ns, x, y, yaw,
                       quality_flags
                FROM route_points
                WHERE mission_id=? AND upload_state='PENDING'
                ORDER BY route_seq ASC LIMIT ?
                ''',
                (mission['mission_id'], limit),
            ).fetchall()
        return mission['mission_id'], [dict(row) for row in rows]

    def has_unsent_route_through(
        self,
        mission_id: str,
        route_end_seq: int,
    ) -> bool:
        with self._lock:
            row = self._connection.execute(
                '''
                SELECT 1 FROM route_points
                WHERE mission_id=? AND route_seq<=?
                  AND upload_state!='SENT'
                LIMIT 1
                ''',
                (mission_id, int(route_end_seq)),
            ).fetchone()
        return row is not None

    def mark_route_batch_sent(
        self,
        mission_id: str,
        route_sequences: Iterable[int],
    ) -> None:
        sequences = tuple(int(value) for value in route_sequences)
        if not sequences:
            return
        placeholders = ','.join('?' for _ in sequences)
        with self._lock, self._connection:
            self._connection.execute(
                f'''
                UPDATE route_points SET upload_state='SENT'
                    , upload_error=''
                WHERE mission_id=? AND route_seq IN ({placeholders})
                ''',
                (mission_id, *sequences),
            )

    def mark_route_batch_blocked(
        self,
        mission_id: str,
        route_sequences: Iterable[int],
        error: str,
    ) -> None:
        sequences = tuple(int(value) for value in route_sequences)
        if not sequences:
            return
        placeholders = ','.join('?' for _ in sequences)
        with self._lock, self._connection:
            self._connection.execute(
                f'''
                UPDATE route_points
                SET upload_state='BLOCKED',
                    upload_attempts=upload_attempts+1,
                    upload_error=?
                WHERE mission_id=? AND route_seq IN ({placeholders})
                  AND upload_state='PENDING'
                ''',
                (error[:1000], mission_id, *sequences),
            )

    def retry_blocked(self) -> int:
        """Move operator-reviewed permanent failures back into the queue."""
        with self._lock, self._connection:
            outbox_result = self._connection.execute(
                '''
                UPDATE outbox
                SET state='PENDING', next_attempt_ns=0, last_error=''
                WHERE state='BLOCKED'
                '''
            )
            route_result = self._connection.execute(
                '''
                UPDATE route_points
                SET upload_state='PENDING', upload_error=''
                WHERE upload_state='BLOCKED'
                '''
            )
            return int(outbox_result.rowcount + route_result.rowcount)

    def pending_counts(self) -> dict[str, int]:
        with self._lock:
            outbox_rows = self._connection.execute(
                '''
                SELECT state, COUNT(*) AS count
                FROM outbox GROUP BY state
                '''
            ).fetchall()
            outbox_by_state = {
                str(row['state']): int(row['count'])
                for row in outbox_rows
            }
            outbox_pending = outbox_by_state.get('PENDING', 0)
            outbox_failed = outbox_by_state.get('FAILED', 0)
            outbox_blocked = outbox_by_state.get('BLOCKED', 0)
            route_rows = self._connection.execute(
                '''
                SELECT upload_state, COUNT(*) AS count
                FROM route_points GROUP BY upload_state
                '''
            ).fetchall()
            route_by_state = {
                str(row['upload_state']): int(row['count'])
                for row in route_rows
            }
            route_pending = route_by_state.get('PENDING', 0)
            route_blocked = route_by_state.get('BLOCKED', 0)
            detections = self._connection.execute(
                'SELECT COUNT(*) AS count FROM detections'
            ).fetchone()['count']
        return {
            'outbox': outbox_pending + outbox_failed + outbox_blocked,
            'outbox_pending': outbox_pending,
            'outbox_failed': outbox_failed,
            'outbox_blocked': outbox_blocked,
            'route_points': route_pending + route_blocked,
            'route_pending': route_pending,
            'route_blocked': route_blocked,
            'detections': int(detections),
        }
