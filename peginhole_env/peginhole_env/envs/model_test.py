import mujoco_py as mp
import numpy as np
import time

class PegInHole():
    def __init__(self):
        self.model = mp.load_model_from_path('fall2020_peginhole_square_ori.xml')
        self.sim = mp.MjSim(self.model)
        self.viewer = mp.MjViewer(self.sim)
        self.error_velocity_XYZ_former = np.array([[0.],[0.],[0.]]) #初值为0，之后根据阻抗控制进行更新
        self.error_velocity_RPY_former = np.array([[0.],[0.],[0.]])

    def admittanceControl(self, Fxd, Fyd, Fzd, Txd, Tyd, Tzd):
        # 导纳参数
        MF = np.array([[0.0, 0., 0.],
                       [0., 0.0, 0.],
                       [0., 0., 0.0]])  
        
        BF = np.array([[1., 0., 0.],
                       [0., 1., 0.],
                       [0., 0., 1.]])  
        
        MT = np.array([[0.0, 0., 0.],
                       [0., 0.0, 0.],
                       [0., 0., 0.0]])  
        
        BT = np.array([[1., 0., 0.],
                       [0., 1., 0.],
                       [0., 0., 1.]])  
        # F_d
        cur_Force_Target = np.array([[Fxd, Fyd, Fzd]]).transpose() 
        cur_Torque_Target = np.array([[Txd, Tyd, Tzd]]).transpose()  
    
        force_torque_read = self.sim.data.cfrc_ext[1, :]
        cur_Force = np.array([force_torque_read[3:6]]).transpose()
        cur_Torque = np.array([force_torque_read[0:3]]).transpose()

        error_force = cur_Force_Target - cur_Force
        error_torque = cur_Torque_Target - cur_Torque

        # v_d
        velocity_XYZ_target = np.array([[0., 0., 0.]]).transpose() 
        velocity_RPY_target = np.array([[0., 0., 0.]]).transpose()

        #导纳控制
        temp_force = error_force + np.matmul(240*MF,self.error_velocity_XYZ_former)
        temp_torque = error_torque + np.matmul(240*MT,self.error_velocity_RPY_former)
        # print(temp_torque[2])
        # print(240*MT+BT)
        # print(np.linalg.inv(240*MT+BT))

        velocity_XYZ = velocity_XYZ_target - np.matmul(np.linalg.inv(240*MF+BF),temp_force)
        velocity_RPY = velocity_RPY_target - np.matmul(np.linalg.inv(240*MT+BT),temp_torque)

        self.error_velocity_XYZ_former = velocity_XYZ_target - velocity_XYZ
        self.error_velocity_RPY_former = velocity_RPY_target - velocity_RPY

        return velocity_XYZ, velocity_RPY

    def run_admittance(self):
        for i in range(3000):
            target_XYZ_send, target_RPY_send = self.admittanceControl(0, 0, -1, 0, 0, 0)
            target_XYZ_send = np.squeeze(target_XYZ_send)
            target_RPY_send = np.squeeze(target_RPY_send)
            # print(self.sim.data.get_body_xpos("peg")[2])
            fullaction = np.concatenate((target_XYZ_send, target_RPY_send))
            print('fullaction', fullaction.transpose().round(3)[2])
            # print('target_XYZ_send', target_XYZ_send.transpose().round(3))
            # print('target_RPY_send', target_RPY_send.transpose().round(3))
            self.sim.data.ctrl[:6] = [target_XYZ_send[0],target_XYZ_send[1],target_XYZ_send[2],target_RPY_send[0],target_RPY_send[1],target_RPY_send[2]]
            self.sim.step()
            self.viewer.render()
            # force = self.sim.data.cfrc_ext[1, :]
            # force_rounded = [round(val, 5) for val in force]
            # print('force_rounded', force_rounded)

    def run(self):
        start = time.time()
        start_pos = self.sim.data.get_body_xpos("peg")[2]
        start_ori = self.sim.data.get_body_xquat("peg")[2]
        for i in range(300):
            self.sim.data.ctrl[:6] = [0, 0, 0, 0, 0, 1]
            self.sim.step()
            self.viewer.render()
            # force = self.sim.data.cfrc_ext[1, :]
            # force_rounded = [round(val, 5) for val in force]
            # print('force_rounded', force_rounded)
            # print(self.sim.data.get_body_xpos("peg")[2])
            end_pos = self.sim.data.get_body_xpos("peg")[2]
            end_ori = self.sim.data.get_body_xquat("peg")[2]
            print("sensor pos vel", self.sim.data.get_body_xvelp("peg"))
            print("sensor ori vel", self.sim.data.get_body_xvelr("peg"))
        end = time.time()
        print("cost time:", end - start)
        print("pos vel:", (end_pos - start_pos)/(end - start))
        print("sensor vel", self.sim.data.get_body_xvelp("peg"))
        print("ori vel:", (end_ori - start_ori)/(end - start))


    
if __name__ == "__main__":
    env = PegInHole()
    env.run_admittance()
