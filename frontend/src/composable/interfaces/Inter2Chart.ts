export interface ChartDataPoint {
    timeStr: string;
    ff: number;
    mf: number;
    th: number;
}

export interface SeriesDataInterface {
    ff: number[];
    mf: number[];
    th: number[];
}