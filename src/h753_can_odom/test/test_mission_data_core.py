import json
import math
from pathlib import Path
import sqlite3

from h753_can_odom.mission_data_core import (
    DetectionEventGate,
    MapManifest,
    MissionStore,
    NavigationGoalTracker,
    NavigationMissionTracker,
    PoseSample,
    decide_route_sample,
)


WORKSPACE_DIR = Path(__file__).resolve().parents[3]
GO2_MAP_YAML = WORKSPACE_DIR / 'maps' / 'go2' / 'go2_map.yaml'


def test_navigation_goal_tracker_emits_each_terminal_result_once() -> None:
    tracker = NavigationGoalTracker()

    assert tracker.observe('goal-a', 1).event == 'started'
    assert tracker.observe('goal-a', 2) is None
    succeeded = tracker.observe('goal-a', 4)

    assert succeeded is not None
    assert succeeded.event == 'succeeded'
    assert tracker.active_goal_id is None
    assert tracker.observe('goal-a', 4) is None


def test_navigation_goal_tracker_distinguishes_failure_and_replacement() -> None:
    tracker = NavigationGoalTracker()

    tracker.observe('goal-a', 2)
    replaced = tracker.observe('goal-b', 1)

    assert replaced is not None
    assert replaced.event == 'replaced'
    assert replaced.previous_goal_id == 'goal-a'
    aborted = tracker.observe('goal-b', 6)
    assert aborted is not None
    assert aborted.event == 'aborted'


def test_any_number_of_waypoint_children_complete_one_mission_at_end() -> None:
    for waypoint_count in (1, 2, 4, 10, 25):
        tracker = NavigationMissionTracker()
        route_id = f'route-{waypoint_count}'

        started = tracker.observe('follow_waypoints', route_id, 1)
        assert started is not None and started.event == 'started'

        for index in range(waypoint_count):
            child_id = f'{route_id}-waypoint-{index}'
            assert tracker.observe('navigate_to_pose', child_id, 1) is None
            assert tracker.observe('navigate_to_pose', child_id, 4) is None
            assert tracker.active_source == 'follow_waypoints'
            assert tracker.active_goal_id == route_id

        completed = tracker.observe('follow_waypoints', route_id, 4)
        assert completed is not None
        assert completed.event == 'succeeded'
        assert tracker.active_source is None


def test_parent_route_promotes_racy_first_child_without_new_mission() -> None:
    tracker = NavigationMissionTracker()

    child = tracker.observe('navigate_to_pose', 'waypoint-0', 1)
    assert child is not None and child.event == 'started'
    parent = tracker.observe('follow_waypoints', 'route-a', 1)

    assert parent is not None
    assert parent.event == 'promoted'
    assert parent.previous_source == 'navigate_to_pose'
    assert tracker.active_source == 'follow_waypoints'
    assert tracker.observe('navigate_to_pose', 'waypoint-0', 4) is None
    assert tracker.observe('follow_waypoints', 'route-a', 4).event == 'succeeded'


def test_amcl_manifest_captures_map_revision_and_pixel_metadata() -> None:
    first = MapManifest.from_amcl_yaml(GO2_MAP_YAML)
    second = MapManifest.from_amcl_yaml(GO2_MAP_YAML)

    assert first.map_id == second.map_id
    assert first.map_id == 'map_e5c5c16c93333f71'
    assert first.backend == 'amcl'
    assert first.resolution == 0.05
    assert first.origin_x == -33.6273
    assert first.origin_y == -50.1284
    assert first.origin_yaw == 0.0
    assert first.width == 1093
    assert first.height == 1596
    assert len(first.image_checksum) == 64


def test_route_decision_filters_small_motion_and_marks_pose_jump() -> None:
    previous = PoseSample(1, 1.0, 2.0, 0.0)

    small = decide_route_sample(
        previous,
        PoseSample(2, 1.01, 2.0, math.radians(1.0)),
        elapsed_since_record_s=0.5,
        min_distance_m=0.05,
        min_yaw_rad=math.radians(5.0),
        max_interval_s=5.0,
        max_pose_jump_m=1.0,
        max_yaw_jump_rad=math.radians(60.0),
    )
    jump = decide_route_sample(
        previous,
        PoseSample(3, 2.5, 2.0, 0.0),
        elapsed_since_record_s=0.5,
        min_distance_m=0.05,
        min_yaw_rad=math.radians(5.0),
        max_interval_s=5.0,
        max_pose_jump_m=1.0,
        max_yaw_jump_rad=math.radians(60.0),
    )

    assert small.record is False
    assert jump.record is True
    assert jump.pose_jump is True


def test_detection_gate_coalesces_person_and_blue_edges_until_gateway_rearm():
    gate = DetectionEventGate(('/person', '/blue'))

    assert gate.update_gate('/person', 1, now=0.0) is True
    assert gate.update_gate('/blue', 1, now=0.1) is False
    gate.update_gateway(False, now=1.0)
    gate.update_gate('/person', 0, now=2.0)
    gate.update_gate('/blue', 0, now=2.0)
    assert gate.update_gateway(True, now=16.0) is True
    assert gate.update_gate('/person', 1, now=17.0) is True


def test_detection_gate_fallback_requires_cooldown_and_clear() -> None:
    gate = DetectionEventGate(
        ('/person', '/blue'),
        fallback_cooldown_s=15.0,
        fallback_clear_s=2.0,
    )
    gate.update_gate('/person', 1, now=0.0)
    gate.update_gate('/person', 0, now=10.0)

    assert gate.tick(now=14.9) is False
    assert gate.tick(now=15.0) is True


def make_manifest() -> MapManifest:
    return MapManifest(
        map_id='map-001',
        backend='amcl',
        frame_id='map',
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        width=10,
        height=10,
    )


def test_store_resumes_mission_and_batches_routes_after_start_ack(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    manifest = make_manifest()
    mission, created = store.create_or_resume_mission(
        'robot-1',
        manifest,
        stamp_ns=1_000_000_000,
        start_mode=4,
    )
    resumed, created_again = store.create_or_resume_mission(
        'robot-1',
        manifest,
        stamp_ns=2_000_000_000,
        start_mode=4,
    )

    assert created is True
    assert created_again is False
    assert resumed['mission_id'] == mission['mission_id']

    map_event = store.next_outbox_event(now_ns=3_000_000_000)
    assert map_event is not None
    assert map_event.event_type == 'map'
    store.mark_outbox_sent(map_event.event_key, 3_000_000_000)
    start_event = store.next_outbox_event(now_ns=3_000_000_000)
    assert start_event is not None
    assert start_event.event_type == 'mission_start'

    first_seq = store.add_route_point(
        mission['mission_id'],
        PoseSample(4_000_000_000, 0.0, 0.0, 0.0),
        segment_id=0,
    )
    assert first_seq == 0
    assert store.pending_route_batch(100) is None

    store.mark_outbox_sent(start_event.event_key, 4_000_000_000)
    route_batch = store.pending_route_batch(100)
    assert route_batch is not None
    route_mission, points = route_batch
    assert route_mission == mission['mission_id']
    assert [point['route_seq'] for point in points] == [0]

    store.mark_route_batch_sent(route_mission, [0])
    assert store.pending_route_batch(100) is None
    store.close()


def test_store_detection_and_image_are_idempotent_outbox_events(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    manifest = make_manifest()
    mission, _ = store.create_or_resume_mission(
        'robot-1',
        manifest,
        stamp_ns=1_000_000_000,
        start_mode=4,
    )
    route_seq = store.add_route_point(
        mission['mission_id'],
        PoseSample(2_000_000_000, 1.25, -0.5, 0.3),
        segment_id=0,
    )
    detection_id, payload = store.add_detection(
        mission['mission_id'],
        manifest,
        PoseSample(2_000_000_000, 1.25, -0.5, 0.3),
        route_seq,
        {'person_found': True},
        image_id='image-1',
        detection_id='detection-1',
        detected_at_ns=1_900_000_000,
    )
    image = tmp_path / 'person.jpg'
    image.write_bytes(b'jpeg-data')

    assert detection_id == 'detection-1'
    assert payload['robot_id'] == 'robot-1'
    assert payload['route_end_seq'] == 0
    assert payload['detected_at_ns'] == 1_900_000_000
    assert payload['robot_pose']['x'] == 1.25
    assert payload['robot_pose']['stamp_ns'] == 2_000_000_000
    assert store.attach_detection_image(
        detection_id,
        'image-1',
        image,
        image_stamp_ns=2_100_000_000,
    ) is True

    counts = store.pending_counts()
    assert counts['detections'] == 1
    assert counts['outbox'] == 4
    with store._lock:
        rows = store._connection.execute(
            'SELECT event_type, payload_json FROM outbox ORDER BY event_type'
        ).fetchall()
    event_types = {row['event_type'] for row in rows}
    assert event_types == {
        'map',
        'mission_start',
        'detection',
        'detection_image',
    }
    image_payload = next(
        json.loads(row['payload_json'])
        for row in rows
        if row['event_type'] == 'detection_image'
    )
    assert len(image_payload['image_checksum']) == 64
    store.close()


def test_ending_mission_queues_end_after_pending_routes(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    mission, _ = store.create_or_resume_mission(
        'robot-1',
        make_manifest(),
        stamp_ns=1,
        start_mode=4,
    )
    ended = store.end_active_mission(2)

    assert ended == mission['mission_id']
    assert store.active_mission() is None
    with store._lock:
        row = store._connection.execute(
            "SELECT payload_json FROM outbox WHERE event_type='mission_end'"
        ).fetchone()
    assert json.loads(row['payload_json'])['status'] == 'COMPLETED'
    store.close()


def test_map_change_aborts_old_mission_with_local_reason(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    old_mission, _ = store.create_or_resume_mission(
        'robot-1',
        make_manifest(),
        stamp_ns=1,
        start_mode=4,
    )
    changed_manifest = MapManifest(
        map_id='map-002',
        backend='amcl',
        frame_id='map',
    )

    new_mission, created = store.create_or_resume_mission(
        'robot-1',
        changed_manifest,
        stamp_ns=2,
        start_mode=4,
    )

    assert created is True
    assert new_mission['mission_id'] != old_mission['mission_id']
    with store._lock:
        old_row = store._connection.execute(
            'SELECT status, end_reason FROM missions WHERE mission_id=?',
            (old_mission['mission_id'],),
        ).fetchone()
        end_payload = json.loads(
            store._connection.execute(
                "SELECT payload_json FROM outbox "
                "WHERE event_key=?",
                (f"mission:{old_mission['mission_id']}:end",),
            ).fetchone()['payload_json']
        )
    assert old_row['status'] == 'ABORTED'
    assert old_row['end_reason'] == 'MAP_CHANGED'
    assert end_payload['status'] == 'ABORTED'
    assert end_payload['end_reason'] == 'MAP_CHANGED'
    store.close()


def test_failed_outbox_event_retries_and_preserves_server_ack(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    store.create_or_resume_mission(
        'robot-1',
        make_manifest(),
        stamp_ns=1,
        start_mode=4,
    )
    event = store.next_outbox_event(now_ns=10)
    assert event is not None

    store.mark_outbox_failed(event.event_key, 'network down', 100)
    counts = store.pending_counts()
    assert counts['outbox_failed'] == 1
    assert counts['outbox_pending'] == 1
    assert store.next_outbox_event(now_ns=99) is None

    retry = store.next_outbox_event(now_ns=100)
    assert retry is not None
    assert retry.event_key == event.event_key
    assert retry.attempts == 1
    store.mark_outbox_sent(
        retry.event_key,
        sent_at_ns=200,
        acknowledgement={'ack': True, 'map_id': 'map-001'},
    )

    with store._lock:
        row = store._connection.execute(
            'SELECT state, ack_json FROM outbox WHERE event_key=?',
            (retry.event_key,),
        ).fetchone()
    assert row['state'] == 'SENT'
    assert json.loads(row['ack_json'])['ack'] is True
    assert store.pending_counts()['outbox_failed'] == 0
    store.close()


def test_permanent_failures_are_blocked_until_operator_requeues(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    mission, _ = store.create_or_resume_mission(
        'robot-1',
        make_manifest(),
        stamp_ns=1,
        start_mode=4,
    )
    event = store.next_outbox_event(now_ns=10)
    assert event is not None
    store.mark_outbox_blocked(event.event_key, 'HTTP 401')

    counts = store.pending_counts()
    assert counts['outbox_blocked'] == 1
    assert store.next_outbox_event(now_ns=100) is None

    route_seq = store.add_route_point(
        mission['mission_id'],
        PoseSample(2, 0.0, 0.0, 0.0),
        segment_id=0,
    )
    store.mark_route_batch_blocked(
        mission['mission_id'],
        [route_seq],
        'HTTP 400',
    )
    counts = store.pending_counts()
    assert counts['route_pending'] == 0
    assert counts['route_blocked'] == 1

    assert store.retry_blocked() == 2
    counts = store.pending_counts()
    assert counts['outbox_blocked'] == 0
    assert counts['route_blocked'] == 0
    assert counts['outbox_pending'] == 2
    assert counts['route_pending'] == 1
    store.close()


def test_detection_waits_for_route_through_detection_sequence(tmp_path):
    store = MissionStore(tmp_path / 'mission.db')
    mission, _ = store.create_or_resume_mission(
        'robot-1',
        make_manifest(),
        stamp_ns=1,
        start_mode=4,
    )
    first = store.add_route_point(
        mission['mission_id'],
        PoseSample(2, 0.0, 0.0, 0.0),
        segment_id=0,
    )
    second = store.add_route_point(
        mission['mission_id'],
        PoseSample(3, 0.1, 0.0, 0.0),
        segment_id=0,
    )

    assert store.has_unsent_route_through(mission['mission_id'], first)
    store.mark_route_batch_sent(mission['mission_id'], [first])
    assert not store.has_unsent_route_through(mission['mission_id'], first)
    assert store.has_unsent_route_through(mission['mission_id'], second)
    store.close()


def test_existing_outbox_schema_adds_ack_column_without_data_loss(tmp_path):
    database = tmp_path / 'legacy.db'
    connection = sqlite3.connect(database)
    connection.execute(
        '''
        CREATE TABLE outbox (
            event_key TEXT PRIMARY KEY,
            event_type TEXT NOT NULL,
            mission_id TEXT NOT NULL,
            payload_json TEXT NOT NULL,
            file_path TEXT NOT NULL DEFAULT '',
            state TEXT NOT NULL DEFAULT 'PENDING',
            attempts INTEGER NOT NULL DEFAULT 0,
            next_attempt_ns INTEGER NOT NULL DEFAULT 0,
            last_error TEXT NOT NULL DEFAULT '',
            created_at_ns INTEGER NOT NULL,
            sent_at_ns INTEGER
        )
        '''
    )
    connection.execute(
        '''
        INSERT INTO outbox(
            event_key, event_type, mission_id, payload_json, created_at_ns
        ) VALUES('legacy-event', 'map', 'legacy-mission', '{}', 1)
        '''
    )
    connection.commit()
    connection.close()

    store = MissionStore(database)
    with store._lock:
        columns = {
            row['name']
            for row in store._connection.execute(
                'PRAGMA table_info(outbox)'
            ).fetchall()
        }
        row = store._connection.execute(
            'SELECT event_key, ack_json FROM outbox '
            "WHERE event_key='legacy-event'"
        ).fetchone()
    assert 'ack_json' in columns
    assert row['event_key'] == 'legacy-event'
    assert row['ack_json'] == ''
    store.close()


def test_existing_mission_and_route_schema_adds_blocking_columns(tmp_path):
    database = tmp_path / 'legacy-routes.db'
    connection = sqlite3.connect(database)
    connection.executescript(
        '''
        CREATE TABLE missions (
            mission_id TEXT PRIMARY KEY,
            robot_id TEXT NOT NULL,
            map_id TEXT NOT NULL,
            map_manifest_json TEXT NOT NULL,
            started_at_ns INTEGER NOT NULL,
            ended_at_ns INTEGER,
            start_mode INTEGER,
            status TEXT NOT NULL,
            created_at_ns INTEGER NOT NULL
        );
        CREATE TABLE route_points (
            mission_id TEXT NOT NULL,
            route_seq INTEGER NOT NULL,
            segment_id INTEGER NOT NULL,
            stamp_ns INTEGER NOT NULL,
            x REAL NOT NULL,
            y REAL NOT NULL,
            yaw REAL NOT NULL,
            quality_flags TEXT NOT NULL DEFAULT '',
            upload_state TEXT NOT NULL DEFAULT 'PENDING',
            PRIMARY KEY (mission_id, route_seq)
        );
        INSERT INTO missions VALUES(
            'm1', 'r1', 'map1', '{}', 1, NULL, 4, 'ACTIVE', 1
        );
        INSERT INTO route_points VALUES(
            'm1', 0, 0, 1, 0.0, 0.0, 0.0, '', 'PENDING'
        );
        '''
    )
    connection.commit()
    connection.close()

    store = MissionStore(database)
    with store._lock:
        mission_columns = {
            row['name'] for row in store._connection.execute(
                'PRAGMA table_info(missions)'
            ).fetchall()
        }
        route_columns = {
            row['name'] for row in store._connection.execute(
                'PRAGMA table_info(route_points)'
            ).fetchall()
        }
        route = store._connection.execute(
            'SELECT upload_state, upload_attempts, upload_error '
            'FROM route_points WHERE mission_id="m1"'
        ).fetchone()
    assert 'end_reason' in mission_columns
    assert {'upload_attempts', 'upload_error'} <= route_columns
    assert route['upload_state'] == 'PENDING'
    assert route['upload_attempts'] == 0
    assert route['upload_error'] == ''
    store.close()
