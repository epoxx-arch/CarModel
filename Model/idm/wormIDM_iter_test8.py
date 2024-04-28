import numpy as np
import matplotlib.pyplot as plt
import math
import copy
import time


class ObsDecision:
    def __init__(self, id, start_t, end_t, start_s_l, start_s_u, end_s_l, end_s_u, speed, if_reverse):
        self.id = id
        self.start_t = start_t
        self.end_t = end_t
        self.start_s_l = start_s_l
        self.start_s_u = start_s_u
        self.end_s_l = end_s_l
        self.end_s_u = end_s_u
        self.speed = speed
        self.extend_speed = 0
        self.step = 0.1
        self.back_extend_t = start_t
        self.back_extend_s_l = start_s_l
        self.back_extend_s_u = start_s_u
        self.extend = False
        self.path_clear = False
        self.if_reverse = if_reverse

    def interpolate_points(self, x1, y1, x2, y2, step):
        points = []
        if self.extend and self.if_reverse == False:
            num_points = round((self.start_t-self.back_extend_t)/step)
            for i in range(num_points+1):
                x = self.back_extend_t + i * self.step
                y = y1
                points.append((x, y, self.extend_speed))
        if self.extend and self.if_reverse == True:
            print("triger reverse process!!!")
            num_points = round((self.start_t-self.back_extend_t)/step)
            slope = (y2 - y1) / (x2 - x1)
            for i in range(num_points+1):
                x = self.back_extend_t + i * self.step
                if y1 == self.start_s_u:
                    y = self.back_extend_s_u + i * self.step * slope
                else:
                    y = self.back_extend_s_l + i * self.step * slope
                points.append((x, y, self.speed))
        num_points = round((x2-x1)/step)
        # self.extend = False
        if x1 == x2:
            slope = None
        else:
            slope = (y2 - y1) / (x2 - x1)

        for i in range(num_points+1):
            if slope is None:
                x = x1
            else:
                x = x1 + i * self.step
            if slope is None:
                y = y1 + (i / (num_points)) * (y2 - y1)
            else:
                y = y1 + (x - x1) * slope
            points.append((x, y, self.speed))
        return points

    def iter_extend_start_t(self):
        if self.start_t == self.end_t:
            print("obs's decision is something wrong")
        if (self.back_extend_t > 0.5):
            # 因为要extend两次所以先改成0.25，需要优化
            self.back_extend_t = self.back_extend_t - 0.5
        else:
            self.back_extend_t = 0
        self.back_extend_s_l = self.start_s_l
        self.back_extend_s_u = self.start_s_u
        if self.if_reverse:
            slope_l = (self.end_s_l - self.start_s_l) / \
                (self.end_t - self.start_t)
            slope_u = (self.end_s_u - self.start_s_u) / \
                (self.end_t - self.start_t)
            theta_t = self.back_extend_t-self.start_t
            theta_s_l = slope_l * theta_t
            theta_s_u = slope_u * theta_t
            self.back_extend_s_l = self.start_s_l + theta_s_l
            self.back_extend_s_u = self.start_s_u + theta_s_u
            print("it is reverse obs")
        self.extend = True
        return self.interpolate_points_for_low(), self.interpolate_points_for_upper()

    def interpolate_points_for_low(self):
        return self.interpolate_points(self.start_t, self.start_s_l, self.end_t, self.end_s_l, self.step)

    def interpolate_points_for_upper(self):
        return self.interpolate_points(self.start_t, self.start_s_u, self.end_t, self.end_s_u, self.step)

    def reset_extend(self):
        self.extend = False


class Hybird_IDM:
    def __init__(self, ego, obs_low_points):
        self.ego_v_start = ego.ego_v
        self.ego_a_start = ego.ego_a
        self.start_ego_p = ego.ego_p
        self.start_theta_s = obs_low_points[0][1]-self.start_ego_p
        self.obs_info = obs_low_points
        self.obs_position = obs_low_points[0][1]
        self.range_start_t = obs_low_points[0][0]
        self.range_end_t = obs_low_points[-1][0]
        self.step = 0.1
        self.P = 0.3
        self.headway = 0.5
        self.default_spacing = 3
        self.desire_max_a = 1.2
        self.desire_min_b = 2

    def CalIdmProcess(self):
        cal_obj_distance = []
        cal_obj_distance .append(self.start_theta_s)
        ego_v = []
        ego_v.append(self.ego_v_start)
        t_cal = []
        t_cal.append(self.range_start_t)
        ego_p = []
        ego_p.append(self.start_ego_p)
        distance_s = 100
        last_acc_cmd = self.ego_a_start
        acc_cmd = []
        acc_cmd.append(last_acc_cmd)
        theta_time = self.step
        obs_p = []
        obs_p.append(self.obs_position)

        for i in range(round((self.range_end_t-self.range_start_t)/0.1)):
            v_obj = self.obs_info[i][2]
            # obs_p.append(self.obs_info[i][1])

            headway_var = self.headway+v_obj/8
            print("headway : ", headway_var)
            S_star = self.default_spacing+headway_var*ego_v[len(ego_v)-1]+ego_v[len(ego_v)-1]*(
                0-(v_obj-ego_v[len(ego_v)-1]))/(2*math.sqrt(self.desire_max_a*self.desire_min_b))
            distance_s = cal_obj_distance[len(cal_obj_distance)-1] + 0.5

            accelerate_cmd = self.desire_max_a * \
                (1+((v_obj-ego_v[len(ego_v)-1])/(ego_v[len(ego_v)-1]+1.5))
                 ** 1-(S_star/distance_s)**2)
            theta_a = self.P * (accelerate_cmd-last_acc_cmd)
            accelerate_cmd = last_acc_cmd + theta_a
            if accelerate_cmd < -4:
                accelerate_cmd = -4
            acc_cmd.append(accelerate_cmd)
            last_acc_cmd = accelerate_cmd
            if ego_v[len(ego_v)-1]+theta_time*accelerate_cmd <= 0:
                ego_v.append(0)
            else:
                ego_v.append(ego_v[len(ego_v)-1]+theta_time*accelerate_cmd)
            ego_p.append(ego_p[len(ego_p)-1]+theta_time*ego_v[len(ego_v)-1])
            t_cal.append(theta_time+t_cal[len(t_cal)-1])
            cal_obj_distance.append(
                self.obs_info[i][1]-ego_p[len(ego_p)-1]+theta_time*ego_v[len(ego_v)-1])
        return ego_v, ego_p, t_cal, acc_cmd


def FindClosestTimeAndIndex(x_values, y):
    min_distance = 10000
    nearest_x = None
    nearest_index = None
    for i, x in enumerate(x_values):
        distance = abs(x-y)
        if distance < min_distance:
            min_distance = distance
            nearest_x = x
            nearest_index = i
    return nearest_x, nearest_index


class CheckCollision:
    def __init__(self, ego_pos_predict, ego_time_cal_predict, obs_decision):
        self.ego_pos_predict = ego_pos_predict
        self.ego_time_cal_predict = ego_time_cal_predict
        self.ego_predict_start_t = ego_time_cal_predict[0]
        self.ego_predict_end_t = ego_time_cal_predict[len(
            ego_time_cal_predict)-1]
        self.obs_decision = obs_decision
        self.collision_buffer = 0.2
        self.step = 0.1

    def FindClosestPoint(self, x_values, y):
        min_distance = 10000
        nearest_x = None
        nearest_index = None
        for i, x in enumerate(x_values):
            distance = abs(x-y)
            if distance < min_distance:
                min_distance = distance
                nearest_x = x
                nearest_index = i
        return nearest_x, nearest_index

    def CalCollisionPoint(self):
        obs_decision_start = self.obs_decision.start_t
        obs_decision_end = self.obs_decision.end_t
        if self.ego_predict_start_t >= obs_decision_end or self.ego_predict_end_t <= obs_decision_start:
            return False, 0
        else:
            time_lap_start = max(self.ego_predict_start_t, obs_decision_start)
            time_lap_end = min(self.ego_predict_end_t, obs_decision_end)
            num_points = round((time_lap_end-time_lap_start)/self.step)
            time_range = np.linspace(
                time_lap_start, time_lap_end, num_points+1)
            point_low_vector = self.obs_decision.interpolate_points_for_low()
            time_elements_low = [item[0] for item in point_low_vector]
            point_up_vector = self.obs_decision.interpolate_points_for_upper()
            time_elements_up = [item[0] for item in point_up_vector]
            has_up_overlap = False
            has_low_overlap = False
            has_overlap = False
            for t in time_range:
                time_ego, time_index_ego = self.FindClosestPoint(
                    self.ego_time_cal_predict, t)
                ego_pos = self.ego_pos_predict[time_index_ego]

                time_obs_low, time_index_obs_low = self.FindClosestPoint(
                    time_elements_low, t)
                pos_obs_low = point_low_vector[time_index_obs_low][1]
                time_obs_up, time_index_obs_up = self.FindClosestPoint(
                    time_elements_up, t)
                pos_obs_up = point_up_vector[time_index_obs_up][1]
                if ego_pos <= (pos_obs_up+1.2) and ego_pos >= (pos_obs_low-1.2):
                    has_overlap = True
                    return has_overlap, time_index_ego
            return False, 0


class EgoSate:
    def __init__(self, ego_v, ego_a, ego_p, ego_t):
        self.ego_v = ego_v
        self.ego_a = ego_a
        self.ego_p = ego_p
        self.ego_t = ego_t


class HybirdPID:
    def __init__(self, ego_v_start, ego_a_start, start_ego_p, range_start_t, range_end_t):
        self.ego_v_start = ego_v_start
        self.ego_a_start = ego_a_start
        self.start_ego_p = start_ego_p
        self.range_start_t = range_start_t
        self.range_end_t = range_end_t
        self.step = 0.1
        self.P = 0.2
        self.v_set = 5
        self.desire_max_a = 1.2

    def CalPIDProcess(self):
        ego_v = []
        ego_v.append(self.ego_v_start)
        obs3 = ObsDecision(2, 6, 7, 13, 19.5, 9.5, 17.5, -2, True)

        t_cal = []
        t_cal.append(self.range_start_t)
        ego_p = []
        ego_p.append(self.start_ego_p)
        last_acc_cmd = self.ego_a_start
        acc_cmd = []
        acc_cmd.append(last_acc_cmd)
        theta_time = self.step

        for i in range(round((self.range_end_t-self.range_start_t)/0.1)):
            accelerate_cmd = self.desire_max_a * \
                (1 - ((ego_v[len(ego_v)-1])/(self.v_set))**2)
            theta_a = self.P * (accelerate_cmd-last_acc_cmd)
            accelerate_cmd = last_acc_cmd + theta_a
            acc_cmd.append(accelerate_cmd)
            last_acc_cmd = accelerate_cmd
            if ego_v[len(ego_v)-1]+theta_time*accelerate_cmd <= 0:
                ego_v.append(0)
            else:
                ego_v.append(ego_v[len(ego_v)-1]+theta_time*accelerate_cmd)
            ego_p.append(ego_p[len(ego_p)-1]+theta_time*ego_v[len(ego_v)-1])
            t_cal.append(theta_time+t_cal[len(t_cal)-1])
        return ego_v, ego_p, t_cal, acc_cmd


def custom_compare(obj):
    return (obj.start_t, obj.start_s_l)


start = time.time()
# id, start_t, end_t, start_s_l, start_s_u, end_s_l, end_s_u, speed
obs1 = ObsDecision(0, 1, 2, 190, 220, 220, 250, 3, False)
obs2 = ObsDecision(1, 3, 4, 15, 18, 12, 13, -3, False)
obs3 = ObsDecision(2, 6, 7, 13, 19.5, 9.5, 17.5, -2, True)

# obs1 = ObsDecision(0, 1, 2, 7, 12, 10, 15, 3, False)
# obs2 = ObsDecision(1, 4, 5, 8, 11.5, 9, 12.5, 1, False)
obs3 = ObsDecision(2, 6, 7, 13, 19.5, 9.5, 17.5, -2, True)
obs4 = ObsDecision(3, 2, 3, 19, 21, 20, 22, 1, False)
obs5 = ObsDecision(4, 0.2, 2, 12, 14, 16, 18, 2, False)
# 障碍物排序
obs_vector = []
obs_vector.append(obs1)
# obs_vector.append(obs2)
# obs_vector.append(obs3)
# obs_vector.append(obs4)
# obs_vector.append(obs5)
sorted_objects = sorted(obs_vector, key=custom_compare)


completion_obs_list = []

if sorted_objects[0].start_t > 0:
    new_obs = ObsDecision(
        0, 0, sorted_objects[0].start_t, 50, 51, 50, 51, 0, False)
    new_obs.path_clear = True
    completion_obs_list.append(new_obs)

max_t = 0
for obj in sorted_objects:
    max_t = max(obj.end_t, max_t)

if max_t < 8:
    new_obs = ObsDecision(
        0, max_t, 8, 50, 51, 50, 51, 0, False)
    new_obs.path_clear = True
    completion_obs_list.append(new_obs)

completion_obs_list = sorted_objects+completion_obs_list
free_gaps = []
current_time = sorted_objects[0].start_t
print("no use?  ", current_time)

for interval in sorted_objects:
    if current_time < interval.start_t:
        gap = (current_time, interval.start_t)
        free_gaps.append(gap)
    current_time = max(current_time, interval.end_t)

for gap in free_gaps:
    new_obs = ObsDecision(
        0, gap[0], gap[1], 50, 51, 50, 51, 0, False)
    new_obs.path_clear = True
    completion_obs_list.append(new_obs)
    # print(f"Start: {gap[0]}, End: {gap[1]}")

final_sorted_objects = sorted(completion_obs_list, key=custom_compare)
print("len of new obs : ", len(final_sorted_objects))
print(len(final_sorted_objects))


for obs in final_sorted_objects:
    upper_line = obs.interpolate_points_for_upper()
    lower_line = obs.interpolate_points_for_low()
    x_coords, y_coords, speed_vector = zip(*upper_line)
    # plt.plot(x_coords, y_coords, marker='o',
    #          linestyle='-', alpha=1, markersize=10)
    x_coords, y_coords, speed_vector = zip(*lower_line)
    # plt.plot(x_coords, y_coords, marker='o',
    #          linestyle='-', alpha=1, markersize=10)

# 按顺序遍历所有障碍物
ego_current_state = EgoSate(10, 0, 0, 0)
caled_ego_v_seris = []
caled_ego_p_seris = []
caled_ego_a_seris = []
accumuted_time = []

# 首先以最大速度推演
pid_process = HybirdPID(ego_current_state.ego_v, ego_current_state.ego_a,
                        ego_current_state.ego_p, 0, 8)
ego_v_orin, ego_p_orin, t_cal_orin, ego_a_orin = pid_process.CalPIDProcess()

caled_ego_v_seris = caled_ego_v_seris + ego_v_orin
caled_ego_a_seris = caled_ego_a_seris + ego_a_orin
caled_ego_p_seris = caled_ego_p_seris + ego_p_orin
accumuted_time = accumuted_time + t_cal_orin
# plt.plot(t_cal_orin, ego_p_orin, marker='o', linestyle='-', color="black")
# plt.plot(t_cal_orin, ego_v_orin, marker='o', linestyle='-', color="red")
# plt.plot(t_cal_orin, ego_a_orin, marker='o', linestyle='-', color="green")
# 遍历所有障碍物
first_has_overlap = False
first_collision_obs_index = 100000

for i in range(len(final_sorted_objects)):
    collision_check = CheckCollision(
        caled_ego_p_seris, accumuted_time, final_sorted_objects[i])
    first_has_overlap, nouse = collision_check.CalCollisionPoint()
    if first_has_overlap:
        print("first has overlap with obs index : ", i)
        first_collision_obs_index = i
        break
print("if has overlap ? : ", first_has_overlap)

if first_has_overlap:
    iter_obs = True
    obs_need_iter_list = [
        (final_sorted_objects[first_collision_obs_index], first_collision_obs_index)]

index_brefore_clear = 10000
iter_index = 0
iter_obs_extend = 0
while first_has_overlap:
    iter_index = iter_index+1
    if len(obs_need_iter_list) > 0:
        print("step into A idm ")
        obs_extend = copy.deepcopy(obs_need_iter_list[-1])
        obs_low_points, obs_up_points = obs_extend[0].iter_extend_start_t()
        iter_obs_extend = iter_obs_extend + 1
        print("checkout iter_obs_extend is : ", iter_obs_extend)
        obs_extend[0].reset_extend()
        x_coords, y_coords, speed = zip(*obs_low_points)
        plt.plot(x_coords, y_coords, marker='o',
                 linestyle='-', alpha=1, markersize=10)
        x_coords, y_coords, speed = zip(*obs_up_points)
        plt.plot(x_coords, y_coords, marker='o',
                 linestyle='-', alpha=1, markersize=10)
        obs_need_iter_list.append(obs_extend)
        nearest_t, nearest_t_index = FindClosestTimeAndIndex(
            accumuted_time, obs_extend[0].back_extend_t)
        print("check last stage start t new : ", nearest_t)
        ego_current_state.ego_v = caled_ego_v_seris[nearest_t_index]
        ego_current_state.ego_a = caled_ego_a_seris[nearest_t_index]
        ego_current_state.ego_p = caled_ego_p_seris[nearest_t_index]
        ego_current_state.ego_t = accumuted_time[nearest_t_index]
        # 截断维护的数据
        caled_ego_a_seris = caled_ego_a_seris[:nearest_t_index]
        caled_ego_v_seris = caled_ego_v_seris[:nearest_t_index]
        caled_ego_p_seris = caled_ego_p_seris[:nearest_t_index]
        accumuted_time = accumuted_time[:nearest_t_index]
        hybird_forward_prodict = Hybird_IDM(ego_current_state, obs_low_points)
        ego_v, ego_p, t_cal, acc_cmd = hybird_forward_prodict.CalIdmProcess()
        # 添加维护数据
        caled_ego_a_seris = caled_ego_a_seris + acc_cmd
        caled_ego_v_seris = caled_ego_v_seris + ego_v
        caled_ego_p_seris = caled_ego_p_seris + ego_p
        accumuted_time = accumuted_time + t_cal
        # 得到最新的推演结果，然后拿过来重新碰撞检测
        # plt.plot(accumuted_time, caled_ego_p_seris,
        #          marker='o', linestyle='-', color="black")
        # plt.plot(accumuted_time, caled_ego_v_seris,
        #          marker='o', linestyle='-', color="red")
        # plt.plot(accumuted_time, caled_ego_a_seris,
        #          marker='o', linestyle='-', color="green")
        obs_has_overlap = False
        # if iter_index >= 6:
        #     break
        collision_obs_index = 10000
        for i in range(len(final_sorted_objects)):
            collision_check = CheckCollision(
                caled_ego_p_seris, accumuted_time, final_sorted_objects[i])
            obs_has_overlap, nouse = collision_check.CalCollisionPoint()
            print("1!!!!!!!!!!!!!!!!!!!", nouse)
            if obs_has_overlap:
                print("A has overlap with obs index : ", i)
                collision_obs_index = i
                if collision_obs_index != obs_extend[1]:
                    print("collision with other obs")
                    obs_need_iter_list.append((
                        final_sorted_objects[collision_obs_index], collision_obs_index))

                break
        print("if has overlap ? : ", obs_has_overlap)
        if obs_has_overlap == False:
            iter_obs_extend = 0
            index_brefore_clear = obs_need_iter_list[-1][1]
            obs_need_iter_list.clear()
        print("see the all iter index 460 : ", iter_index)

        if iter_obs_extend >= 4:
            print("over the most iter obs expend !", iter_obs_extend)
            print("print len of list", len(obs_need_iter_list))
            print("stop at index : ", collision_obs_index)
            collision_check = CheckCollision(
                caled_ego_p_seris, accumuted_time, final_sorted_objects[collision_obs_index])
            has_overlap, nearest_t_index = collision_check.CalCollisionPoint()
            print("cut time at : ", nearest_t_index)
            print("1!!!!!!!!!!!!!!!!!!!", nearest_t_index)
            # # # 先找到碰撞点然后再截断数据
            if nearest_t_index > 15:
                nearest_t_index = nearest_t_index - 10

            caled_ego_a_seris = caled_ego_a_seris[:nearest_t_index]
            caled_ego_v_seris = caled_ego_v_seris[:nearest_t_index]
            caled_ego_p_seris = caled_ego_p_seris[:nearest_t_index]
            accumuted_time = accumuted_time[:nearest_t_index]
            break
    else:
        print("deal with future obs")
        # 先拿到车辆当前状态
        print("update ego state")
        ego_current_state.ego_v = caled_ego_v_seris[-1]
        ego_current_state.ego_a = caled_ego_a_seris[-1]
        ego_current_state.ego_p = caled_ego_p_seris[-1]
        ego_current_state.ego_t = accumuted_time[-1]

        # 判断障碍物类型,这里这个10000未来待优化奥
        if index_brefore_clear != 10000 and final_sorted_objects[index_brefore_clear+1].path_clear == True:
            print("Welcome to PID")
            # ego_v_start, ego_a_start, start_ego_p, range_start_t, range_end_t
            pid_execute = HybirdPID(
                ego_current_state.ego_v, ego_current_state.ego_a, ego_current_state.ego_p, final_sorted_objects[index_brefore_clear+1].start_t, final_sorted_objects[index_brefore_clear+1].end_t)
            ego_v, ego_p, t_cal, acc_cmd = pid_execute.CalPIDProcess()
            caled_ego_a_seris = caled_ego_a_seris + acc_cmd
            caled_ego_v_seris = caled_ego_v_seris + ego_v
            caled_ego_p_seris = caled_ego_p_seris + ego_p
            accumuted_time = accumuted_time + t_cal
            # plt.plot(accumuted_time, caled_ego_p_seris,
            #          marker='o', linestyle='-', color="black")
            # plt.plot(accumuted_time, caled_ego_v_seris,
            #          marker='o', linestyle='-', color="red")
            # plt.plot(accumuted_time, caled_ego_a_seris,
            #          marker='o', linestyle='-', color="green")
            index_brefore_clear = index_brefore_clear+1
            print("PID process finished, next index is ", index_brefore_clear)
            print("see len is ", len(final_sorted_objects))
            if index_brefore_clear+1 >= len(final_sorted_objects):
                break
            # 注意，PID情况，是不需要进行碰撞检测的
        elif index_brefore_clear != 10000 and final_sorted_objects[index_brefore_clear+1].path_clear == False:
            print("Welcome to Hybird IDM B")
            obs_concern = final_sorted_objects[index_brefore_clear+1]
            obs_points_low = obs_concern.interpolate_points_for_low()
            # 不管怎么样吧，都要截断操作
            nearest_t, nearest_t_index = FindClosestTimeAndIndex(
                accumuted_time, obs_points_low[0][0])
            print("check last stage start t new : ", nearest_t)
            ego_current_state.ego_v = caled_ego_v_seris[nearest_t_index]
            ego_current_state.ego_a = caled_ego_a_seris[nearest_t_index]
            ego_current_state.ego_p = caled_ego_p_seris[nearest_t_index]
            ego_current_state.ego_t = accumuted_time[nearest_t_index]
            # 截断维护的数据
            caled_ego_a_seris = caled_ego_a_seris[:nearest_t_index]
            caled_ego_v_seris = caled_ego_v_seris[:nearest_t_index]
            caled_ego_p_seris = caled_ego_p_seris[:nearest_t_index]
            accumuted_time = accumuted_time[:nearest_t_index]

            hybird_forward_prodict = Hybird_IDM(
                ego_current_state, obs_points_low)
            ego_v, ego_p, t_cal, acc_cmd = hybird_forward_prodict.CalIdmProcess()
            caled_ego_a_seris = caled_ego_a_seris + acc_cmd
            caled_ego_v_seris = caled_ego_v_seris + ego_v
            caled_ego_p_seris = caled_ego_p_seris + ego_p
            accumuted_time = accumuted_time + t_cal
            # 得到最新的推演结果，然后拿过来重新碰撞检测
            # plt.plot(accumuted_time, caled_ego_p_seris,
            #          marker='o', linestyle='-', color="black")
            # plt.plot(accumuted_time, caled_ego_v_seris,
            #          marker='o', linestyle='-', color="red")
            # plt.plot(accumuted_time, caled_ego_a_seris,
            #          marker='o', linestyle='-', color="green")
            obs_has_overlap = False
            print("process idm  with obs index : ", index_brefore_clear+1)
            for i in range(len(final_sorted_objects)):
                collision_check = CheckCollision(
                    caled_ego_p_seris, accumuted_time, final_sorted_objects[i])
                obs_has_overlap, nouse = collision_check.CalCollisionPoint()
                # print("B has overlap with obs index : ", i)
                # print("test ! ", obs_has_overlap)
                if obs_has_overlap:
                    collision_obs_index = i
                    if collision_obs_index != index_brefore_clear+1:
                        # 这样好像如果撞上之前的车了，那放最后再次处理，可能就死循环了，待优化
                        obs_need_iter_list.append(
                            final_sorted_objects[collision_obs_index], collision_obs_index)
                    break
            print("if has overlap ? : ", obs_has_overlap)
            if obs_has_overlap:
                obs_need_iter_list.append((obs_concern, index_brefore_clear+1))
                print("B idm has overlap , send to list")
            if obs_has_overlap == False:
                index_brefore_clear = index_brefore_clear + 1
                obs_need_iter_list.clear()
            if index_brefore_clear+1 >= len(final_sorted_objects):
                break
    print("see the all iter index 565 : ", iter_index)
    if iter_index >= 16:
        break
end = time.time()
print("time use : ", end-start)
plt.plot(accumuted_time, caled_ego_p_seris,
         marker='o', linestyle='-', color="black")
plt.plot(accumuted_time, caled_ego_v_seris,
         marker='o', linestyle='-', color="red")
plt.plot(accumuted_time, caled_ego_a_seris,
         marker='o', linestyle='-', color="green")
plt.ylim(-5, 52)
plt.xlabel('T Coordinate')
plt.ylabel('S Coordinate')
plt.title('cilqr speed planner')
plt.grid(True)
plt.show()
