if __name__ == '__main__':
    # copy the following codes outside this package to run
    from articulate.utils.imu948 import IMU948DotSet
    from articulate.utils.bullet import RotationViewer
    from articulate.math import quaternion_to_rotation_matrix
    from pygame.time import Clock

    imus_addr_mac = [
    'C09A717C-7830-BBF4-C294-3079E86778FC', #im948_A-V3.02
    '4C0E565D-5456-3743-21C0-35837B2E8044', #im948_C-V3.02
    '3165FEBF-F328-4820-B901-C1DF260F3BED', #im948_D-V3.02
    '4396544E-E3E6-4BB0-D0FF-DC860D1C387D', #im948_E-V3.02
    '476A1419-BE11-4697-69AB-BE6664A4595E', #im948_H-V3.02
    'E2699DC5-BCFB-B63A-B709-B410F3EEC93F', #im948_I-V3.02
]

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
    clock = Clock()

    IMU948DotSet.sync_connect(imus_addr_mac)
    # IMU948DotSet.reset_xyz()
    IMU948DotSet.start_streaming()
    with RotationViewer(len(imus_addr_mac)) as viewer:
        IMU948DotSet.clear()
        for _ in range(60 * 60):  # 10s
            for i in range(len(imus_addr_mac)):
                clock.tick(60)
                t, q, a = IMU948DotSet.get(i)
                print("\rfps:", clock.get_fps()," t:",t, " q:",q, " a:",a, end='')
                viewer.update(quaternion_to_rotation_matrix(q).view(3, 3), i)
    IMU948DotSet.sync_disconnect()
