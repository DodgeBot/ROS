acc = [0, 0, 0]

velocity = [[0, 0, 0], [0, 0, 0]]
acceleration = [[0, 0, 0], [0, 0, 0]]

calibrate_count = 0
calibrate_count_num = 100

calibrated = False

acc_calibrate_sum = [0, 0, 0]

filter_count = 0
filter_count_num = 3

x_acc_pub = rospy.Publisher('imu_acc_x', Float64, queue_size=10)
y_acc_pub = rospy.Publisher('imu_acc_y', Float64, queue_size=10)
z_acc_pub = rospy.Publisher('imu_acc_z', Float64, queue_size=10)

x_v_pub = rospy.Publisher('imu_v_x', Float64, queue_size=10)
y_v_pub = rospy.Publisher('imu_v_y', Float64, queue_size=10)
z_v_pub = rospy.Publisher('imu_v_z', Float64, queue_size=10)

def calibrate():
    global calibrate_count, acc_calibrate_sum, acc, calibrate_count_num, calibrated
    if calibrated:
        return
    while calibrate_count <= calibrate_count_num:
        for i in range(3):
            acc_calibrate_sum[i] = acc_calibrate_sum[i] + acc[i]
        calibrate_count = calibrate_count + 1
    for i in range(3):
        acc_calibrate_sum[i] = acc_calibrate_sum[i] / calibrate_count_num
    calibrate_count = 0
    calibrated = True

def average_filter():
    global acc_filter_sum, filter_count, filter_count_num, acceleration, acc
    while filter_count <= filter_count_num:
        for i in range(3):
            acceleration[0][i] = acceleration[0][i] + acc[i]
        filter_count = filter_count + 1
    filter_count = 0
    for i in range(3):
        acceleration[0][i] = acceleration[0][i] / filter_count_num

movement_end_check_count = [0, 0, 0]
movement_end_check_count_num = 25
def movement_end_check():
    global acceleration, velocity, movement_end_check_count
    for i in range(3):
        if abs(acceleration[0][i]) < 0.009:
            movement_end_check_count[i] = movement_end_check_count[i] + 1
        else:
            movement_end_check_count[i] = 0
        if movement_end_check_count[i] >= movement_end_check_count_num:
            velocity[0][i] = 0

def publish_message():
    # x_acc_pub.publish(acceleration[0][0])
    # y_acc_pub.publish(acceleration[0][1])
    # z_acc_pub.publish(acceleration[0][2])

    x_acc_pub.publish(acc[0])
    y_acc_pub.publish(acc[1])
    z_acc_pub.publish(acc[2])
    #
    # x_v_pub.publish(velocity[0][0])
    # y_v_pub.publish(velocity[0][1])
    # z_v_pub.publish(velocity[0][2])

def get_velocity():
    calibrate()
    average_filter()
    for i in range(3):
        acceleration[0][i] = acceleration[0][i] - acc_calibrate_sum[i]
        #discrimination window
        if abs(acceleration[0][i]) < 0.009:
            acceleration[0][i] = 0
        #integration
        velocity[0][i] = velocity[1][i] + acceleration[1][i] + (acceleration[0][i] - acceleration[1][i])/2
        #update
        acceleration[1][i] = acceleration[0][i]
        velocity[1][i] = velocity[0][i]

        movement_end_check()
        publish_message()
