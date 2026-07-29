# planner.py

import config_hanson


class Planner:


    def __init__(self):

        print("Planner 起動")

        self.steering = 0
        self.throttle = 0



    def update(self, perception):

        """
        perception.pyの結果を受け取り
        ステアリング・速度を決定する

        return:
            steering (-1.0 ~ 1.0)
            throttle (0 ~ 1.0)
        """


        if perception is None:

            return 0, 0



        # =========================
        # 緊急回避
        # =========================

        if perception["front_wall"]:


            print("前方障害物")


            left = perception["left_distance"]
            right = perception["right_distance"]



            if left is not None and right is not None:


                if left > right:

                    # 左側が空いている
                    self.steering = -1


                else:

                    # 右側が空いている
                    self.steering = 1


            else:

                self.steering = 0



            self.throttle = config_hanson.CORNER_SPEED


            return self.steering, self.throttle




        # =========================
        # 走行モード選択
        # =========================


        if config_hanson.HAND_SIDE == "left":

            return self.left_hand(perception)



        else:

            return self.right_hand(perception)






    # =========================
    # 左手法
    # =========================

    def left_hand(self, perception):


        left = perception["left_distance"]


        if left is None:

            return 0,0



        target = config_hanson.TARGET_DISTANCE


        error = left - target



        if error > 100:

            # 壁が遠い
            steering = -1



        elif error < -100:

            # 壁が近い
            steering = 1



        else:

            steering = 0




        speed = config_hanson.STRAIGHT_SPEED



        return steering, speed






    # =========================
    # 右手法
    # =========================

    def right_hand(self, perception):


        right = perception["right_distance"]


        if right is None:

            return 0,0



        target = config_hanson.TARGET_DISTANCE



        error = right - target




        if error > 100:

            # 壁が遠い
            steering = 1



        elif error < -100:

            # 壁が近い
            steering = -1



        else:

            steering = 0




        speed = config_hanson.STRAIGHT_SPEED



        return steering, speed
