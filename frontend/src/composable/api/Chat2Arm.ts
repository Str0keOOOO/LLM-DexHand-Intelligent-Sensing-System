import request from '@/composable/utils/request'
import type {TrackScriptRequest, SysVarRequest, ArmJointPositions} from '@/composable/types/robot';

export const armApi = {
    // 连接与初始化
    connect: () => request.get('/robotic_arm/connect'),
    initRobot: () => request.post('/robotic_arm/init_robot'),

    // 脚本与状态
    callTrackScript: (data: TrackScriptRequest) => request.post('/robotic_arm/call_track_script', data),
    checkProgramState: () => request.get<{ status: string; data: any }>('/robotic_arm/check_program_state'),
    getCurrentLine: () => request.get<{ line: number }>('/robotic_arm/get_current_line'),

    // 夹爪控制
    openGripper: () => request.post('/robotic_arm/open_gripper'),
    closeGripper: () => request.post('/robotic_arm/close_gripper'),

    // 数据获取
    getActualJointPos: () => request.get<ArmJointPositions>('/robotic_arm/get_actual_joint_pos_degree'),
    getSysVarValue: (data: SysVarRequest) => request.post<{ value: any }>('/robotic_arm/get_sys_var_value', data)
};