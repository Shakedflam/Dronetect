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


def test_camera_gimbal_sets_and_clamps_absolute_pitch():
    gimbal = CameraGimbal()

    assert gimbal.set_pitch(0.0) == 0.0
    assert gimbal.set_pitch(2.0) == 1.7
    assert gimbal.set_pitch(-2.0) == -0.5


def test_camera_gimbal_clamps_to_safe_pitch_limits():
    gimbal = CameraGimbal(pitch=1.69)

    assert gimbal.tilt_down() == 1.7
    assert gimbal.tilt_down() == 1.7

    gimbal = CameraGimbal(pitch=-0.49)

    assert gimbal.tilt_up() == -0.5
    assert gimbal.tilt_up() == -0.5
