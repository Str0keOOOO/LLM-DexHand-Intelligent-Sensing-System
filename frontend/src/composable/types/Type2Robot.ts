export type Hand = 'left' | 'right'

export type Joints = {
    th_dip: number; th_mcp: number; th_rot: number
    ff_spr: number; ff_dip: number; ff_mcp: number
    mf_dip: number; mf_mcp: number
    rf_dip: number; rf_mcp: number
    lf_dip: number; lf_mcp: number
}

export type ControlForm = {
    hand: Hand
    joints: Joints
}