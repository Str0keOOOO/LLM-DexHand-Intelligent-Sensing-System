class systemConfig:
    # 定义所有的变量
    def __init__(self):
        self.PLC_IP = '192.168.5.10'
        # self.robotSerialPort = 'COM14'
        self.laserPrinterFilePath = '{"F":"ODC","DP":"D:\\\\software\\\\test\\\\webModbus\\\\BslCAD2.orzx"}#'
        self.laserPrinterSignaturePath = '{"F": "SPI", "DC": "BslCAD2.orzx","M": 1,"P":' + \
                                         '[{"N":0,"T":"456","FH":10,"FP":"D:\\\\software\\\\test\\\\webModbus\\\\static\\\\upload\\\\signature.svg",' \
                                         '"X":-10,"Y":0,"W":10,"H":10,"R":10,"FT":"Arial","FL":0}]}' + '#'
        ## 心跳信号地址
        self.plcAddress = "opc.tcp://192.168.5.10:4840"
        self.robotAddress = '192.168.5.4'
        self.updateMesOperationUrl = "http://localhost:8080/mes/workOrder/renewJobs"
        # self.updateMesOperationUrl = "http://192.168.5.125:8080/mes/workOrder/renewJobs"
        self.robotPort = 502
        self.plcHeartBeatAddress = {
            "value": 'ns=4;i=15',
            "des": "PLC心跳信号"
        }
        self.agvHeartBeatAddress = {
            "value": 'ns=4;i=22',
            "des": "AGV心跳信号"
        }
        self.robotHeartBeatAddress = {
            "value": 'ns=4;i=36',
            "des": "机器人心跳信号"
        }
        self.cncHeartBeatAddress = {
            "value": 'ns=4;i=46',
            "des": "CNC心跳信号"
        }
        self.laserPrinterHeartBeatAddress = {
            "value": 'ns=4;i=66',
            "des": "打标机心跳信号"
        }
        self.agvExecuteRequest = {
            "value": "ns=4;i=26",
            "des": "AGV请求执行"
        }
        self.agvExecuteSuccess = {
            "value": "ns=4;i=33",
            "des": "AGV执行成功"
        }
        self.agvDeviceNumber = {
            "value": "ns=4;i=27",
            "des": "AGV设备编号"
        }
        self.agvActionSingle = {
            "value": "ns=4;i=28",
            "des": "AGV请求动作类型"
        }
        self.agvStatusSingle = {
            "value": "ns=4;i=29",
            "des": "AGV状态信号"
        }
        self.agvError = "ns=4;i=34"

        self.robotExecuteRequest = {
            "value": "ns=4;i=38",
            "des": "PLC请求运动机器人"
        }
        self.robotExecuteSuccess = {
            "value": "ns=4;i=43",
            "des": "机器人执行成功"
        }
        self.robotDeviceNumber = {
            "value": "ns=4;i=39",
            "des": "机器人设备编号"
        }
        self.robotActionSingle = {
            "value": "ns=4;i=40",
            "des": "机器人请求动作类型"
        }
        self.robotStatusSingle = {
            "value": "ns=4;i=41",
            "des": "机器人状态信号"
        }
        self.plcNoticeGrippersStart = {
            "value": "ns=4;i=75",
            "des": "通知PLC启动机器人夹紧或关闭"
        }
        self.plcResultGrippers = {
            "value": "ns=4;i=76",
            "des": "PLC返回机器人夹具结果(已经夹紧或关闭)"
        }

        self.asyncGrippersSingle = {
            "value": "ns=4;i=76",
            "des": "读取PLC中夹具信号-夹紧False, 打开True "
        }

        self.robotError = "ns=4;i=44"

        self.cncExecuteRequest = {
            "value": "ns=4;i=48",
            "des": "CNC请求执行"
        }
        self.cncExecuteSuccess = {
            "value": "ns=4;i=53",
            "des": "CNC执行成功"
        }
        self.cncDeviceNumber = {
            "value": "ns=4;i=49",
            "des": "CNC设备编号"
        }
        self.cncActionSingle = {
            "value": "ns=4;i=50",
            "des": "CNC请求动作类型"
        }
        self.cncStatusSingle = {
            "value": "ns=4;i=51",
            "des": "CNC状态信号"
        }
        self.cncError = "ns=4;i=54"

        self.photoExecuteRequest = {
            "value": "ns=4;i=58",
            "des": "拍照请求执行"
        }
        self.photoExecuteSuccess = {
            "value": "ns=4;i=63",
            "des": "拍照执行成功"
        }
        self.photoDeviceNumber = {
            "value": "ns=4;i=59",
            "des": "拍照设备编号"
        }
        self.photoActionSingle = {
            "value": "ns=4;i=60",
            "des": "拍照请求动作类型"
        }
        self.photoStatusSingle = {
            "value": "ns=4;i=61",
            "des": "拍照状态信号"
        }
        self.photoError = "ns=4;i=64"

        self.printerExecuteRequest = {
            "value": "ns=4;i=68",
            "des": "打标请求执行"
        }
        self.printerExecuteSuccess = {
            "value": "ns=4;i=73",
            "des": "打标执行成功"
        }
        self.printerDeviceNumber = {
            "value": "ns=4;i=69",
            "des": "打标设备编号"
        }
        self.printerActionSingle = {
            "value": "ns=4;i=70",
            "des": "拍照请求动作类型"
        }
        self.printerStatusSingle = {
            "value": "ns=4;i=71",
            "des": "打标状态信号"
        }
        self.printerError = "ns=4;i=74"

        self.productType = {
            "value": "ns=4;i=14",
            "des": "产品类型"
        }
        self.orderNumber = {
            "value": "ns=4;i=17",
            "des": "工单号"
        }
        self.warehouse = {
            "value": "ns=4;i=16",
            "des": "出库库位号"
        }
        self.operationStart = {
            "value": "ns=4;i=13",  # Boolean
            "des": "订单开始"
        }
        self.operationEnd = {
            "value": "ns=4;i=18",
            "des": "订单结束"
        }

        self.angleA1 = {
            "value": "ns=4;i=77",
            "des": "机器人A1角度"
        }

        self.angleA2 = {
            "value": "ns=4;i=78",
            "des": "机器人A2角度"
        }

        self.angleA3 = {
            "value": "ns=4;i=79",
            "des": "机器人A3角度"
        }

        self.angleA4 = {
            "value": "ns=4;i=80",
            "des": "机器人A4角度"
        }

        self.angleA5 = {
            "value": "ns=4;i=81",
            "des": "机器人A5角度"
        }

        self.angleA6 = {
            "value": "ns=4;i=82",
            "des": "机器人A6角度"
        }

        self.agvForwardCommand = "$ZNXJ!"
        self.agvBackwardCommand = "$RZNXJ!"
        self.agvStopCommand = "$TZ!"

        self.robotStartTrack1Command = "00 00 00 00 00 06 01 05 00 04 FF 00"
        self.robotCheckTrack1Command = "00 00 00 00 00 06 01 01 00 15 00 01"
        self.robotReturnTrack1Command = "00 00 00 00 00 06 01 01 00 04 00 01"
        self.robotPositionTrack1Command = "00 00 00 00 00 06 01 05 00 15 00 00"

        self.robotStartTrack2Command = "00 00 00 00 00 06 01 05 00 05 FF 00"
        self.robotCheckTrack2Command = "00 00 00 00 00 06 01 01 00 16 00 01"
        self.robotReturnTrack2Command = "00 00 00 00 00 06 01 01 00 05 00 01"
        self.robotPositionTrack2Command = "00 00 00 00 00 06 01 05 00 16 00 00"

        self.robotStartTrack3Command = "00 00 00 00 00 06 01 05 00 06 FF 00"
        self.robotCheckTrack3Command = "00 00 00 00 00 06 01 01 00 17 00 01"
        self.robotReturnTrack3Command = "00 00 00 00 00 06 01 01 00 06 00 01"
        self.robotPositionTrack3Command = "00 00 00 00 00 06 01 05 00 17 00 00"

        self.robotStartTrack4Command = "00 00 00 00 00 06 01 05 00 07 FF 00"
        self.robotCheckTrack4Command = "00 00 00 00 00 06 01 01 00 18 00 01"
        self.robotReturnTrack4Command = "00 00 00 00 00 06 01 01 00 07 00 01"
        self.robotPositionTrack4Command = "00 00 00 00 00 06 01 05 00 18 00 00"

        self.robotStartTrack5Command = "00 00 00 00 00 06 01 05 00 08 FF 00"
        self.robotCheckTrack5Command = "00 00 00 00 00 06 01 01 00 19 00 01"
        self.robotReturnTrack5Command = "00 00 00 00 00 06 01 01 00 08 00 01"
        self.robotPositionTrack5Command = "00 00 00 00 00 06 01 05 00 19 00 00"

        self.robotStartTrack6Command = "00 00 00 00 00 06 01 05 00 09 FF 00"
        self.robotCheckTrack6Command = "00 00 00 00 00 06 01 01 00 1A 00 01"
        self.robotReturnTrack6Command = "00 00 00 00 00 06 01 01 00 09 00 01"
        self.robotPositionTrack6Command = "00 00 00 00 00 06 01 05 00 1A 00 00"

        self.robotStartTrack7Command = "00 00 00 00 00 06 01 05 00 0A FF 00"
        self.robotCheckTrack7Command = "00 00 00 00 00 06 01 01 00 1B 00 01"
        self.robotReturnTrack7Command = "00 00 00 00 00 06 01 01 00 0A 00 01"
        self.robotPositionTrack7Command = "00 00 00 00 00 06 01 05 00 1B 00 00"

        self.robotStartTrack8Command = "00 00 00 00 00 06 01 05 00 0B FF 00"
        self.robotCheckTrack8Command = "00 00 00 00 00 06 01 01 00 1C 00 01"
        self.robotReturnTrack8Command = "00 00 00 00 00 06 01 01 00 0B 00 01"
        self.robotPositionTrack8Command = "00 00 00 00 00 06 01 05 00 1C 00 00"

        self.robotCloseCommand = "00 00 00 00 00 06 01 05 00 14 FF 00"  # 夹爪夹紧
        self.robotOpenCommand = "00 00 00 00 00 06 01 05 00 14 00 00"  # 夹爪打开

        self.robotInitY14 = "00 00 00 00 00 06 01 05 00 04 00 00"
        self.robotInitY15 = "00 00 00 00 00 06 01 05 00 05 00 00"
        self.robotInitY16 = "00 00 00 00 00 06 01 05 00 06 00 00"
        self.robotInitY17 = "00 00 00 00 00 06 01 05 00 07 00 00"
        self.robotInitY20 = "00 00 00 00 00 06 01 05 00 08 00 00"
        self.robotInitY21 = "00 00 00 00 00 06 01 05 00 09 00 00"
        self.robotInitY22 = "00 00 00 00 00 06 01 05 00 0A 00 00"
        self.robotInitY23 = "00 00 00 00 00 06 01 05 00 0B 00 00"

        # self.agvBleakAddress = "98:da:b0:10:69:7f"
        # self.agvBleakAddress = "98:da:b0:10:6a:b9"
        self.agvBleakAddress = "98:da:b0:10:69:7f"
        self.characteristic_uuid = "0000ffe2-0000-1000-8000-00805f9b34fb"