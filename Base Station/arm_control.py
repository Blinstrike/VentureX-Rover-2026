

# Vertical (Axa 1) - Mișcare braț sus/jos
                                val_1 = utils.map_range(controller.get_axis(1), -1.00, 1.00, -5.00, 5.00)
                                if val_1 > 0.0 and val_1 != last_arm_val_1:
                                    packet = {
                                        "type": "arm",
                                        "cmd": "up",
                                        "val": val_1
                                        }
                                    last_arm_val_1 = val_1
                                    comm.send_msg(sock_container[0], packet)
                                elif val_1 < 0.0 and val_1 != last_arm_val_1:
                                    packet = {
                                        "type": "arm",
                                        "cmd": "down",
                                        "val": val_1
                                    }
                                    last_arm_val_1 = val_1
                                    comm.send_msg(sock_container[0], packet)
                                val_0 = utils.map_range(controller.get_axis(0), -1, 1, -5, 5)
                                if val_0 > 0.0 and val_0 != last_arm_val_0:
                                    packet = {
                                        "type": "arm",
                                        "cmd": "right",
                                        "val": val_0
                                    }
                                    last_arm_val_0 = val_0
                                    comm.send_msg(sock_container[0], packet)
                                elif val_0 < 0.0 and val_0 != last_arm_val_0:
                                    packet = {
                                        "type": "arm",
                                        "cmd": "left",
                                        "val": val_0
                                    }
                                    last_arm_val_0 = val_0
                                    comm.send_msg(sock_container[0], packet)
                                val_2 = utils.map_range(controller.get_axis(2), -1, 1, -5, 5)
                                if val_2 > 0.0 and val_2 != last_arm_val_2:
                                    packet = {
                                        "type": "arm",
                                        "cmd": "forward",
                                        "val": val_2
                                    }
                                    last_arm_val_2 = val_2
                                    comm.send_msg(sock_container[0], packet)
                                elif val_2 < 0.0 and val_2 != last_arm_val_2:
                                    packet = {
                                        "type": "arm",
                                        "cmd": "backward",
                                        "val": val_2
                                    }
                                    last_arm_val_2 = val_2
                                    comm.send_msg(sock_container[0], packet)

