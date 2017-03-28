import threading

import time

from RULEngine.Communication.protobuf import messages_robocup_ssl_wrapper_pb2 as ssl_wrapper

class VisionFrameMerger():

    def __init__(self, vision):
        self.vision = vision
        self.current_merged_frame = ssl_wrapper.SSL_WrapperPacket()
        self.last_frame_number = 0
        self.last_frame_timestamp = 0

        self.cameras_last_dict_frame = [None, None, None, None, None, None]

        self.terminate = threading.Event()
        self.comm_thread = threading.Thread(target=self.merge_loop)
        self.comm_thread.start()

    def merge_loop(self):
        while not self.terminate.is_set():
            frame = self.vision.get_latest_frame()
            if frame != None and frame.detection.frame_number != self.last_frame_number:
                # Keep the last frame of each camera
                self.last_frame_number = frame.detection.frame_number
                if self.cameras_last_dict_frame[frame.detection.camera_id] != None:
                    if frame.detection.t_capture > self.cameras_last_dict_frame[frame.detection.camera_id]['t_capture']:
                        self.cameras_last_dict_frame[frame.detection.camera_id] = self._create_dict_frame(frame)
                else:
                    self.cameras_last_dict_frame[frame.detection.camera_id] = self._create_dict_frame(frame)
                if frame.detection.t_capture > self.last_frame_timestamp:
                    self.last_frame_timestamp = frame.detection.t_capture

                new_merged_dict_frame = {}
                # Merge the last frames into a single frame
                for frame in self.cameras_last_dict_frame:
                    if frame != None:
                        new_merged_dict_frame = self._merge_dict_frames(new_merged_dict_frame, frame)

                new_merged_dict_frame['frame_number'] = self.last_frame_number
                new_merged_dict_frame['t_capture'] = self.last_frame_timestamp
                new_merged_dict_frame['t_sent'] = self.last_frame_timestamp
                new_merged_dict_frame['camera_id'] = 0

                self.current_merged_frame = self._create_frame(new_merged_dict_frame)

                time.sleep(0.001)

    def get_merged_frame(self):
        return self.current_merged_frame

    def _merge_dict_frames(self, frame1, frame2):
        merged_frame = {}
        merged_frame['balls'] = []
        merged_frame['robots_yellow'] = []
        merged_frame['robots_blue'] = []

        try:
            for ball_2 in frame2['balls']:
                new_robot_flag = True
                try:
                    for ball_1 in frame1['balls']:
                        if ball_2.robot_id == ball_1.robot_id:
                            new_robot_flag = False
                            if ball_2.confidence > ball_1.confidence:
                                merged_frame['balls'].append(ball_2)
                            else:
                                merged_frame['balls'].append(ball_1)
                            break
                except KeyError:
                    pass
                if new_robot_flag:
                    merged_frame['balls'].append(ball_2)
        except KeyError:
            pass

        try:
            for robot_yellow_2 in frame2['robots_yellow']:
                new_robot_flag = True
                try:
                    for robot_yellow_1 in frame1['robots_yellow']:
                        if robot_yellow_2.robot_id == robot_yellow_1.robot_id:
                            new_robot_flag = False
                            if robot_yellow_2.confidence > robot_yellow_1.confidence:
                                merged_frame['robots_yellow'].append(robot_yellow_2)
                            else:
                                merged_frame['robots_yellow'].append(robot_yellow_1)
                            break
                except KeyError:
                    pass
                if new_robot_flag:
                    merged_frame['robots_yellow'].append(robot_yellow_2)
        except KeyError:
            pass

        try:
            for robot_blue_2 in frame2['robots_blue']:
                new_robot_flag = True
                try:
                    for robot_blue_1 in frame1['robots_blue']:
                        if robot_blue_2.robot_id == robot_blue_1.robot_id:
                            new_robot_flag = False
                            if robot_blue_2.confidence > robot_blue_1.confidence:
                                merged_frame['robots_blue'].append(robot_blue_2)
                            else:
                                merged_frame['robots_blue'].append(robot_blue_1)
                            break
                except KeyError:
                    pass
                if new_robot_flag:
                    merged_frame['robots_blue'].append(robot_blue_2)
        except KeyError:
            pass

        return merged_frame

    def _create_dict_frame(self, frame):
        dict_frame = {}
        dict_frame['frame_number'] = frame.detection.frame_number
        dict_frame['t_capture'] = frame.detection.t_capture
        dict_frame['t_sent'] = frame.detection.t_sent
        dict_frame['camera_id'] = frame.detection.camera_id
        dict_frame['balls'] = []
        dict_frame['robots_yellow'] = []
        dict_frame['robots_blue'] = []

        for ball in frame.detection.balls:
            dict_frame['balls'].append(ball)
        for robot in frame.detection.robots_yellow:
            dict_frame['robots_yellow'].append(robot)
        for robot in frame.detection.robots_blue:
            dict_frame['robots_blue'].append(robot)

        return dict_frame

    def _create_frame(self, dict_frame):
        frame = ssl_wrapper.SSL_WrapperPacket()

        frame.detection.frame_number = int(dict_frame['frame_number'])
        frame.detection.t_capture = dict_frame['t_capture']
        frame.detection.t_sent = dict_frame['t_sent']
        frame.detection.camera_id = dict_frame['camera_id']

        try:
            for ball in dict_frame['balls']:
                new_ball = frame.detection.balls.add()
                new_ball.confidence = ball.confidence
                new_ball.x = ball.x
                new_ball.y = ball.y
                new_ball.z = ball.z
                new_ball.pixel_x = ball.pixel_x
                new_ball.pixel_y = ball.pixel_y
        except KeyError:
            pass

        try:
            for robot in dict_frame['robots_yellow']:
                new_robot = frame.detection.robots_yellow.add()
                new_robot.robot_id = robot.robot_id
                new_robot.confidence = robot.confidence
                new_robot.x = robot.x
                new_robot.y = robot.y
                new_robot.orientation = robot.orientation
                new_robot.pixel_x = robot.pixel_x
                new_robot.pixel_y = robot.pixel_y
        except KeyError:
            pass

        try:
            for robot in dict_frame['robots_blue']:
                new_robot = frame.detection.robots_blue.add()
                new_robot.robot_id = robot.robot_id
                new_robot.confidence = robot.confidence
                new_robot.x = robot.x
                new_robot.y = robot.y
                new_robot.orientation = robot.orientation
                new_robot.pixel_x = robot.pixel_x
                new_robot.pixel_y = robot.pixel_y
        except KeyError:
            pass

        return frame