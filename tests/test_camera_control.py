from controllers.autonomous_drone.camera_control import CameraGimbal


def test_camera_gimbal_starts_at_default_pitch():
    gimbal = CameraGimbal()

    assert gimbal.pitch == 0.7
    assert gimbal.step == 0.03


def test_camera_gimbal_tilts_up_and_down():
    gimbal = CameraGimbal()

    up_pitch = gimbal.tilt_up()
    down_pitch = gimbal.tilt_down()

    assert up_pitch < 0.7
    assert down_pitch == 0.7


def test_camera_gimbal_clamps_to_safe_pitch_limits():
    gimbal = CameraGimbal(pitch=0.99)

    assert gimbal.tilt_down() == 1.0
    assert gimbal.tilt_down() == 1.0

    gimbal = CameraGimbal(pitch=-1.39)

    assert gimbal.tilt_up() == -1.4
    assert gimbal.tilt_up() == -1.4
