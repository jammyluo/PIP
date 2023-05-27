if __name__ == '__main__':
    # copy the following codes outside this package to run
    from articulate.utils.imu948 import IMU948DotSet
    from articulate.utils.bullet import RotationViewer
    from articulate.math import quaternion_to_rotation_matrix
    imus = [
        # 'D4:22:CD:00:36:80',
        # 'D4:22:CD:00:36:04',
        # 'D4:22:CD:00:32:3E',
        # 'D4:22:CD:00:35:4E',
        # 'D4:22:CD:00:36:03',
        # 'D4:22:CD:00:44:6E',
        # 'D4:22:CD:00:45:E6',
        '2c:01:6e:17:62:3e',
        'e2:62:8b:65:ac:fe',
    ]
    IMU948DotSet.sync_connect(imus)
    IMU948DotSet.start_streaming()
    # IMU948DotSet.reset_heading()
    with RotationViewer(len(imus)) as viewer:
        IMU948DotSet.clear()
        for _ in range(60 * 60):  # 10s
            for i in range(len(imus)):
                t, q, a = IMU948DotSet.get(i)
                print("t:%d,q:%s,a:%s",t,q,a)
                viewer.update(quaternion_to_rotation_matrix(q).view(3, 3), i)
    IMU948DotSet.sync_disconnect()
