!auto_drive && !update_compass && check_position && !continue_moving:
    set_servo_to_105();
    movement_can_buf[2] = -52;
    update_compass = 1;

!auto_drive && !update_compass && !check_position && continue_moving:
    set_servo_to_105();
    continue_moving = 0;

!auto_drive && !update_compass && check_position && continue_moving:
    disable_motor_thread();

!auto_drive && !update_compass && !check_position && !continue_moving:
    disable_motor_thread();

!auto_drive && update_compass:
    set_servo_to_10();
    update_compass = 0;
    continue_moving = 1;

auto_drive && check_position:
    // when the distance is within 7m
    if ( !get_distance_delta_from_current_pos(7, current_latitude, current_longitude) )
        return;
    // continue to run until the distance exceeds 7m
    print_and_update_gps_position(&lat_and_long);
    lat_and_longitude = lat_and_long;
    check_position = 0;
    prev_angle = current_angle;
    if ( global_turn_right )
        turn_angle = 70.0;
    else
        turn_angle = -70.0;
    new_angle = add_floats(current_angle, turn_angle);
    update_compass = 1;

auto_drive && !check_position && update_compass:
    current_angle = compass_update_angle();
    if ( current_angle - dst_angle <= 30 )
    {
        if ( !is_wider_angle(init_angle, dst_angle, current_angle) )
        {
            set_servo_to_105(); // straight
            update_compass = 0;
            update_gps = 1;
            return;
        }
        turn_right = 1;
    }
    else
    {
        turn_right = global_turn_right;
    }
    turn_wheel_servo_to_left_or_right(turn_right);
    movement_can_buf[2] = -52;

auto_drive && !check_position && !update_compass && update_gps:
    print_and_update_gps_position(&lat_and_long);
    lat_and_longitude = lat_and_long;
    update_gps = 0;
    current_latitude = lat_and_long.latitude;
    current_longitude = lat_and_long.longitude;
    flags.continue_moving = 1;


auto_drive && !check_position && !update_compass && !update_gps && continue_moving:
    movement_can_buf[2] = -16;
    set_servo_to_105(); // wheel is 90 degree, straight
    if ( get_distance_delta_from_pre_pos(7, pre_latitude, pre_longitude) )
    {
        disable_motor_thread();
        flags.continue_moving = 0;
        auto_drive = 0;
    }




disable_motor_thread:
    movement_can_buf[2] = 0;
    motor_thread->enabled = 0;


GPS, compass, 55/66/77, global_turn_right

motor_thread->enabled: 55 -> 1, 66 -> 0, 77 -> 1, manual_control over 4999 -> 1


auto_drive: 66 -> 0. when it's 77, it will be set to 1.
check_position: 55 -> 1, 66 -> 0, 77 -> 1

55:
  check_position -> 1

66:
  auto_drive -> 0
  check_position -> 0

77:
  auto_drive -> 1
  check_postion -> 1


after 4999 of manual_control -> 0, auto_drive-> 1.

auto_drive: 0
check_position: 0
update_compass: 0
continue_moving: 0

class ManualActions:
    s2_66: bool = False
    s3_66: bool = False
    s5_66: bool = False
    s7_55: bool = False