#!/usr/bin/env python3
import serial
import time

# 配置串口（核心：与STM32一致）
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200  # 改为你的STM32串口波特率

def read_stm32_data():
    try:
        # 打开串口
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=1.0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print(f"✅ 已打开串口 {SERIAL_PORT}，波特率 {BAUDRATE}")
        print("📥 开始读取STM32数据（按Ctrl+C停止）：\n")
        
        while True:
            # 读取一行数据（STM32需以\n结尾，如printf("data:%d\n", value);）
            data = ser.readline()
            if data:
                # 解码并打印（处理中文/乱码可换gbk编码）
                print(f"STM32发送：{data.decode('utf-8').strip()}")
            time.sleep(0.01)  # 降低CPU占用

    except serial.SerialException as e:
        print(f"❌ 串口错误：{e}")
    except KeyboardInterrupt:
        print("\n🛑 停止读取，关闭串口")
        ser.close()
    except Exception as e:
        print(f"❌ 其他错误：{e}")

if __name__ == "__main__":
    read_stm32_data()

