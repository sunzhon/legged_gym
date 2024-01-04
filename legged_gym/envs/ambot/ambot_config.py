# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class AmbotRoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.21] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_joint1': 0.0,   # [rad]
            'FR_joint1': 0.0 ,  # [rad]
            'HL_joint1': 0.0,   # [rad]
            'HR_joint1': 0.0,   # [rad]

            'FL_joint2': -0.4,     # [rad]
            'FR_joint2': 0.4,   # [rad]
            'HL_joint2': -0.4,     # [rad]
            'HR_joint2': 0.4,   # [rad]

            #'FL_joint3': 0.0,   # [rad]
            #'FR_joint3': 0.0,    # [rad]
            #'HL_joint3': 0.0,  # [rad]
            #'HR_joint3': 0.0,    # [rad]

            'FL_joint4': -0.6,   # [rad]
            'FR_joint4': 0.6,    # [rad]
            'HL_joint4': -0.6,  # [rad]
            'HR_joint4': 0.6,    # [rad]

           # 'head_joint1': .0,   # [rad]
           # 'head_joint2': .0,   # [rad]
           # 'spine_joint1': .0,   # [rad]
           # 'spine_joint2': .0,   # [rad]
           # 'tail_joint1': .0,   # [rad]
           # 'tail_joint2': .0,   # [rad]
           # 'tail_joint3': .0,   # [rad]
           # 'tail_joint4': .0,   # [rad]
        }

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'trimesh'

    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 235
        num_privileged_obs = None # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 12
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds


    class commands(LeggedRobotCfg.commands):
        '''
        operation commands
        '''
        curriculum = False
        max_curriculum = 1.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = True # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-0.5, 0.5] # min max [m/s]
            lin_vel_y = [-0.5, 0.5]   # min max [m/s]
            ang_vel_yaw = [-0.5, 0.5]    # min max [rad/s]
            heading = [-1, 1]

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 40.0 }  # [N*m/rad]
        damping = {'joint': 1.5 }     # [N*m*s/rad] applying_torque=stiff*pos_error+damp*vel_error
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        use_actuator_network = False

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/ambot/urdf/ambot.urdf'
        name = "ambot"
        foot_name = "foot"
        penalize_contacts_on = ["link3", "link2","link1"]
        terminate_after_contacts_on = ["back_shell", "base_link"]
        collapse_fixed_joints = True # merge bodies connected by fixed joints. Specific fixed joints can be kept by adding " <... dont_collapse="true">
        fix_base_link = False # fixe the base of the robot
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = True # Some .obj meshes must be flipped from y-up to z-up
        replace_cylinder_with_capsule = True # replace collision cylinders with capsules, leads to faster/more stable simulation
  

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_base_mass = True
        added_mass_range = [-1, 1]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 0.2


    class rewards( LeggedRobotCfg.rewards ):
        base_height_target = 0.21
        max_contact_force = 100
        class scales( LeggedRobotCfg.rewards.scales ): #define the reward scales which introdues to reward functions
            pass


class AmbotRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_ambot'
        max_iterations = 3000 # number of policy updates

