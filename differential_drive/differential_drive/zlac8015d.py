
from pymodbus.client.sync import ModbusSerialClient as ModbusClient # type: ignore
import numpy as np

import sys
from pathlib import Path
import os
import time 
import pymodbus
class ZLAC8015D_API:
    def __init__(self, port, self_node):
        self._port = port
        self.client = ModbusClient(method='rtu', port=self._port, baudrate=115200, 
                                   stopbits=1,parity="N",bytesize=8,timeout=1)
        self.self_node = self_node
        
        self.was_connected = False 
        self.last_attempt_time = 0  # ⏳ Thời điểm thử kết nối lần cuối
        self.reconnect_interval = 5  # ⏳ Chỉ thử kết nối lại sau 5 giây
        

        self.connected = self.client.connect()
       
        if self.connected:
            print( str(self.connected) + " ==> " + "successfully")
        else:
            print(str(self.connected) + " " + "Failed connection")
            print("Please recheck parameters again!")

        self.ID = 1

        
        ## Common
        self.CONTROL_REG = 0x200E # {0x05: emergency stop, 0x06: clear stop, 0x07: stop, 0x08: enable}
        self.OPR_MODE = 0x200D     # control mode
        self.L_ACL_TIME = 0x2080
        self.R_ACL_TIME = 0x2081
        self.L_DCL_TIME = 0x2082
        self.R_DCL_TIME = 0x2083

        # clear feedback position
        self.CLR_FB_POS = 0x2005

        ## Velocity control
        # Target velocity Range: -3000~3000r/min
        self.L_CMD_RPM = 0x2088
        self.R_CMD_RPM = 0x2089

        # Position control
        self.L_FB_POS_HI = 0x20A7

        self.POS_CONTROL_TYPE = 0x200F  # Synchronous/asynchronous control status

        # Actual velocity  unit: 0.1r/min
        self.L_FB_RPM = 0x20AB
        self.R_FB_RPM = 0x20AC



        ########################
		## Control CMDs (REG) ##
		########################

        self.EMER_STOP = 0x05 # emergency stop
        self.ALRM_CLR = 0x06 # clear default 
        self.DOWN_TIME = 0x07 # stop
        self.ENABLE = 0x08    # enable

        self.POS_SYNC = 0x10
        self.POS_L_START = 0x11
        self.POS_R_START = 0x12


        ####################
		## Operation Mode ##
		####################

        self.POS_REL_CONTROL = 1
        self.POS_ABS_CONTROL = 2
        self.VEL_CONTROL = 3

        self.ASYNC = 0
        self.SYNC = 1


        ## Troubleshooting
        self.L_FAULT = 0x20A5
        self.R_FAULT = 0x20A6

        #################
		## Fault codes ##
		#################

        self.NO_FAULT = 0x0000       # No error
        self.OVER_VOLT = 0x0001      # Over voltage
        self.UNDER_VOLT = 0x0002     # Under voltage
        self.OVER_CURR = 0x0004      # Over current
        self.OVER_LOAD = 0x0008      # Over load
        self.CURR_OUT_TOL = 0x0010   # Current out of tolerance
        self.ENCOD_OUT_TOL = 0x0020  # Encoder out of tolerance
        self.MOTOR_BAD = 0x0040      # Velocity out of tolerance
        self.REF_VOLT_ERROR = 0x0080 # Reference voltage error
        self.EEPROM_ERROR = 0x0100   # EEPROM error
        self.WALL_ERROR = 0x0200     # Hall error
        self.HIGH_TEMP = 0x0400      # Motor temperature over temperature

        self.FAULT_LIST = [self.OVER_VOLT, self.UNDER_VOLT, self.OVER_CURR, self.OVER_LOAD, self.CURR_OUT_TOL, self.ENCOD_OUT_TOL, \
					self.MOTOR_BAD, self.REF_VOLT_ERROR, self.EEPROM_ERROR, self.WALL_ERROR, self.HIGH_TEMP]
        
        ##############
		## Odometry ##
		##############

        ## 4 inches wheel
        self.travel_in_one_rev = 0.336 # distance 
        self.cpr = 4096   #15 poles 
        self.R_Wheel = 0.0535 #0.107 # meter

    # def is_connected(self):
    #     """🔍 Kiểm tra trạng thái kết nối và thử kết nối lại nếu cần"""
    #     if not self.client:
    #         return False

    #     try:
    #         # Thử đọc trạng thái lỗi để kiểm tra kết nối
    #         result = self.client.read_holding_registers(self.L_FAULT, 1, unit=self.ID)
    #         if result is not None and not result.isError():
    #             if not self.was_connected:
    #                 print("✅ Kết nối Modbus đã khôi phục!")  # 🔥 Chỉ in khi kết nối lại
    #                 self.was_connected = True  # Cập nhật trạng thái
    #             return True
    #     except:
    #         pass

    #     # Nếu mất kết nối và trước đó vẫn đang kết nối thì in cảnh báo
    #     if self.was_connected:
    #         print("⚠ Mất kết nối Modbus! Đang thử kết nối lại...")
    #         self.was_connected = False  # Đánh dấu mất kết nối

    #     # 🔄 Thử kết nối lại nếu mất kết nối
    #     if self.client.connect():
    #         print("✅ Kết nối lại Modbus thành công!")  # 🔥 In khi kết nối lại
    #         self.was_connected = True  # Cập nhật trạng thái
    #         return True

    #     return False
    
    def is_connected(self):
        """🔍 Kiểm tra trạng thái kết nối và thử kết nối lại nếu cần"""
        if not self.client:
            return False

        try:
            result = self.client.read_holding_registers(self.L_FAULT, 1, unit=self.ID)
            if result is not None and not result.isError():
                if not self.was_connected:
                    print("✅ Kết nối Modbus đã khôi phục!")
                    self.was_connected = True
                return True
        except:
            pass

        # ⏳ Giới hạn số lần thử lại
        now = time.time()
        if self.was_connected and (now - self.last_attempt_time > self.reconnect_interval):
            print("⚠ Mất kết nối Modbus! Đang thử kết nối lại...")
            self.last_attempt_time = now
            self.was_connected = False

        if self.client.connect():
            print("✅ Kết nối lại Modbus thành công!")
            self.was_connected = True
            return True

        return False



    def reset_motor_after_reconnect(self):
        """🔄 Reset trạng thái động cơ sau khi kết nối lại"""
        try:
            if not self.is_connected():
                print("⚠ Mất kết nối Modbus! Đang thử kết nối lại...")
                self.client.connect()
                time.sleep(1)

            if self.is_connected():
                print("✅ Kết nối lại thành công! Reset trạng thái động cơ...")

                for cmd, desc in [
                    (self.ALRM_CLR, "Clear Alarm"),
                    (self.ENABLE, "Enable Motor"),
                    ((0, 0), "Set RPM = 0")
                ]:
                    try:
                        if isinstance(cmd, tuple):
                            self.client.write_registers(self.L_CMD_RPM, list(cmd), unit=self.ID)
                        else:
                            self.client.write_register(self.CONTROL_REG, cmd, unit=self.ID)
                        time.sleep(0.5)
                    except Exception as e:
                        print(f"❌ Lỗi khi gửi lệnh {desc}: {str(e)}")
                        return  # ❌ Nếu lỗi, dừng lại ngay

                print("✅ Động cơ đã sẵn sàng để điều khiển!")
            else:
                print("❌ Không thể kết nối lại. Hãy kiểm tra dây kết nối hoặc nguồn!")
        except Exception as e:
            print(f"❌ Lỗi khi reset động cơ: {str(e)}")

    def modbus_fail_read_handler(self, ADDR, WORD, max_retries=3, delay=0.1):
        """🔍 Đọc dữ liệu Modbus với tối đa `max_retries` lần thử lại"""
        reg = [None] * WORD  # Mảng chứa dữ liệu đọc được

        for attempt in range(max_retries):
            try:
                result = self.client.read_holding_registers(ADDR, WORD, unit=self.ID)

                # ✅ Kiểm tra lỗi trước khi xử lý dữ liệu
                if result and not result.isError():
                    reg = result.registers  # Gán dữ liệu đọc được
                    return reg  # ✅ Thành công, trả về kết quả

                print(f"⚠️ [Thử lần {attempt+1}/{max_retries}] Lỗi khi đọc Modbus: {result}")

            except AttributeError as e:
                print(f"❌ [Thử lần {attempt+1}/{max_retries}] AttributeError: {e}")

            except Exception as e:
                print(f"❌ [Thử lần {attempt+1}/{max_retries}] Lỗi không xác định: {e}")

            time.sleep(delay)  # ⏳ Chờ trước khi thử lại

        print(f"❌ Không thể đọc địa chỉ Modbus {ADDR} sau {max_retries} lần thử!")
        return reg  # Trả về giá trị mặc định nếu thất bại

    
    def rpm_to_radPerSec(self, rpm):
        return rpm*2*np.pi/60.0
    
    def get_angular_velocity(self):
        rpmL, rpmR = self.get_rpm()
        angular_L =  self.rpm_to_radPerSec(rpmL)
        angular_R = self.rpm_to_radPerSec(rpmR)
        return -angular_L, angular_R
        
    
    def rpm_to_linear(self, rpm):
        W_Wheel = self.rpm_to_radPerSec(rpm)
        V = W_Wheel*self.R_Wheel
        return V
    
    def set_mode(self, MODE):
        if MODE == 1:
            print("Set relative position mode")
        elif MODE == 2:
            print("Set absolute position mode")
        elif MODE == 3:
            print("Set velocity mode")
        else:
            print("set_mode ERROR: set only 1, 2, or 3")
            return 0
        
        result = self.client.write_register(self.OPR_MODE, MODE, unit=self.ID)
        return result

    def clear_position(self, pos):
        if pos not in [0, 1, 2, 3]:  # Kiểm tra giá trị hợp lệ
            print("❌ set_mode POS: chỉ được đặt 0, 1, 2, hoặc 3")
            return False

        messages = {
            1: "✅ Reset vị trí encoder (Left)",
            2: "✅ Reset vị trí encoder (Right)",
            3: "✅ Reset vị trí encoder (Cả hai bên)"
        }

        print(messages.get(pos, "❌ Giá trị pos không hợp lệ"))

        try:
            # Viết giá trị vào thanh ghi Modbus
            self.client.write_register(self.CLR_FB_POS, pos, unit=self.ID)
            return True  # Trả về True nếu không có lỗi
        except Exception as e:
            print(f"❌ Lỗi khi gửi lệnh reset encoder: {str(e)}")
            return False  # Trả về False nếu có lỗi


          


        mode = registers[0]
        return mode
    def enable_motor(self):
        result = self.client.write_register(self.CONTROL_REG, self.ENABLE, unit=self.ID)       
        
        
    def disable_motor(self):
        result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit=self.ID)
    
    def emergency_stop(self):
        result = self.client.write_register(self.CONTROL_REG, self.EMER_STOP, unit=self.ID)
    def get_default(self): # Clear Alarm
        result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR, unit=self.ID)
        
    # def get_fault_code(self):
    #     fault_codes = self.client.read_holding_registers(self.L_FAULT, 2, unit=self.ID)
    #     L_fault_code = fault_codes.registers[0];R_fault_code = fault_codes.registers[1]
    #     L_fault_flag = L_fault_code in self.FAULT_LIST; R_fault_flag = R_fault_code in self.FAULT_LIST
    #     return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)

    def get_fault_code(self):
        """🔍 Lấy mã lỗi của động cơ, có cơ chế thử lại nếu đọc thất bại"""
        try:
            fault_codes = self.modbus_fail_read_handler(self.L_FAULT, 2)  # Sử dụng hàm đọc có thử lại

            # ✅ Kiểm tra xem dữ liệu có hợp lệ không
            if fault_codes is None or len(fault_codes) < 2:
                print("⚠️ Lỗi: Không thể đọc mã lỗi từ Modbus!")
                return (False, None), (False, None)  # Trả về trạng thái lỗi mặc định

            # ✅ Đọc mã lỗi của hai động cơ
            L_fault_code, R_fault_code = fault_codes[0], fault_codes[1]
            L_fault_flag, R_fault_flag = L_fault_code in self.FAULT_LIST, R_fault_code in self.FAULT_LIST

            return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)

        except Exception as e:
            print(f"❌ Lỗi không xác định khi đọc mã lỗi: {str(e)}")
            return (False, None), (False, None)  # Trả về trạng thái lỗi mặc định


    
    # def set_accel_time(self, L_ms, R_ms):
    #     if L_ms > 32767 : L_ms = 32767
    #     elif L_ms < 0 : L_ms = 0

    #     if R_ms > 32767 : R_ms = 32767
    #     elif R_ms < 0 : R_ms = 0

    #     result = self.client.write_registers(self.L_ACL_TIME, [int(L_ms),int(R_ms)], unit=self.ID)

    # def set_decel_time(self, L_ms, R_ms):
    #     if L_ms > 32767 : L_ms = 32767
    #     elif L_ms < 0: L_ms = 0

    #     if R_ms > 32767 : R_ms = 32767
    #     elif R_ms < 0 : R_ms = 0

    #     result = self.client.write_registers(self.L_DCL_TIME, [int(L_ms), int(R_ms)], unit=self.ID)

    def set_accel_time(self, L_ms, R_ms):
        """🔧 Đặt thời gian tăng tốc cho động cơ (ms)"""
        if not self.is_connected():
            print("⚠️ Không thể đặt thời gian tăng tốc: Mất kết nối Modbus!")
            return

        # ✅ Giới hạn giá trị trong khoảng hợp lệ
        L_ms = max(0, min(32767, L_ms))
        R_ms = max(0, min(32767, R_ms))

        try:
            self.client.write_registers(self.L_ACL_TIME, [int(L_ms), int(R_ms)], unit=self.ID)
            print(f"✅ Thời gian tăng tốc đặt thành công: L={L_ms} ms, R={R_ms} ms")
        except Exception as e:
            print(f"❌ Lỗi khi đặt thời gian tăng tốc: {str(e)}")

    def set_decel_time(self, L_ms, R_ms):
        """🔧 Đặt thời gian giảm tốc cho động cơ (ms)"""
        if not self.is_connected():
            print("⚠️ Không thể đặt thời gian giảm tốc: Mất kết nối Modbus!")
            return

        # ✅ Giới hạn giá trị trong khoảng hợp lệ
        L_ms = max(0, min(32767, L_ms))
        R_ms = max(0, min(32767, R_ms))

        try:
            self.client.write_registers(self.L_DCL_TIME, [int(L_ms), int(R_ms)], unit=self.ID)
            print(f"✅ Thời gian giảm tốc đặt thành công: L={L_ms} ms, R={R_ms} ms")
        except Exception as e:
            print(f"❌ Lỗi khi đặt thời gian giảm tốc: {str(e)}")


    
    # def get_rpm(self):
    #     registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)
    #     fb_L_rpm = np.int16(registers[0])/10.0
    #     fb_R_rpm = np.int16(registers[1])/10.0

    #     return fb_L_rpm, fb_R_rpm
    
    def get_rpm(self):
        """📊 Lấy tốc độ quay của động cơ (RPM)"""
        if not self.is_connected():
            print("⚠️ Không thể lấy tốc độ RPM: Mất kết nối Modbus!")
            return 0.0, 0.0  # 🚨 Trả về giá trị mặc định để tránh lỗi

        registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)

        # 🔍 Kiểm tra nếu đọc dữ liệu thất bại
        if registers is None or len(registers) < 2:
            print("❌ Lỗi: Không thể đọc RPM từ động cơ!")
            return 0.0, 0.0  # 🚨 Trả về giá trị mặc định để tránh lỗi

        try:
            fb_L_rpm = np.int16(registers[0]) / 10.0
            fb_R_rpm = np.int16(registers[1]) / 10.0
            return fb_L_rpm, fb_R_rpm
        except (IndexError, TypeError) as e:
            print(f"❌ Lỗi khi xử lý dữ liệu RPM: {str(e)}")
            return 0.0, 0.0  # 🚨 Trả về giá trị mặc định để tránh lỗi




    def set_rpm(self, L_rpm, R_rpm):    # dieu khien dong co
        """✅ Gửi lệnh điều khiển RPM, đảm bảo thử kết nối lại nếu mất kết nối"""        
        if not self.is_connected():  # Sử dụng is_connected() đã sửa
            print("⚠ Không thể gửi lệnh RPM vì Modbus chưa kết nối!")
            return  # ❌ Không gửi lệnh khi chưa có kết nối

        # ✅ Giới hạn tốc độ trong khoảng an toàn
        L_rpm = max(min(L_rpm, 200), -200)
        R_rpm = max(min(R_rpm, 200), -200)
   

        left_bytes = self.int16Dec_to_int16Hex(L_rpm)
        right_bytes = self.int16Dec_to_int16Hex(R_rpm)

        try:
            self.client.write_registers(self.L_CMD_RPM, [left_bytes, right_bytes], unit=self.ID)
        except pymodbus.exceptions.ConnectionException as e:
            print(f"❌ Lỗi kết nối Modbus khi gửi set_rpm: {str(e)}")
        except Exception as e:
            print(f"❌ Lỗi không xác định khi gửi set_rpm: {str(e)}")
    
 
    def int16Dec_to_int16Hex(self, int16):
        """✅ Chuyển đổi số nguyên 16-bit từ dạng thập phân sang hex"""
        try:
            
            int16 = np.int16(int16)  # Đảm bảo giá trị hợp lệ
            # int16 = int(int16) 
            lo_byte = int16 & 0x00FF
            hi_byte = (int16 & 0xFF00) >> 8
       
            return int((hi_byte << 8) | lo_byte)
        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi chuyển đổi số: {str(e)}")
            return None

    # def int16Dec_to_int16Hex(self, int16):
    #     """✅ Chuyển đổi số nguyên 16-bit từ dạng thập phân sang hex"""
    #     try:
    #         # int16 = np.int16(int16)  # Đảm bảo giá trị hợp lệ
    #         lo_byte = int16 & 0x00FF
    #         hi_byte = (int16 & 0xFF00) >> 8
    #         return (hi_byte << 8) | lo_byte
    #     except Exception as e:
    #         self.self_node.get_logger().error(f"❌ Lỗi khi chuyển đổi số: {str(e)}")
    #         return None
        


    def stop_motor_emergency(self):
        """🛑 Gửi lệnh Emergency Stop để đảm bảo động cơ dừng ngay lập tức"""
        try:
            if not self.is_connected():  # 🔍 Kiểm tra trạng thái kết nối
                self.self_node.get_logger().warn("⚠ Mất kết nối Modbus! Đang thử kết nối lại...")
                for _ in range(5):  # 🔄 Thử lại tối đa 5 lần
                    if self.client.connect():
                        self.self_node.get_logger().info("✅ Kết nối lại Modbus thành công!")
                        break
                    time.sleep(0.5)  # ⏳ Chờ 500ms rồi thử lại

            if self.is_connected():  # 🔍 Chỉ gửi lệnh nếu đã kết nối lại thành công
                self.client.write_register(self.CONTROL_REG, self.EMER_STOP, unit=self.ID)
                self.self_node.get_logger().info("🛑 Động cơ đã nhận lệnh Emergency Stop!")
            else:
                self.self_node.get_logger().error("❌ Không thể kết nối lại Modbus. Hãy kiểm tra dây hoặc nguồn!")

        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi gửi Emergency Stop: {str(e)}")





    def clear_alarm(self):
        """🛑 Xóa cảnh báo lỗi trên bộ điều khiển"""
        if not self.is_connected():
            self.self_node.get_logger().error("❌ Mất kết nối! Không thể gửi lệnh clear_alarm().")
            return False

        result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR, unit=self.ID)

        if result.isError():
            self.self_node.get_logger().error("❌ Không thể xóa lỗi! Kiểm tra kết nối và thử lại.")
            return False

        self.self_node.get_logger().info("✅ Đã xóa lỗi trên bộ điều khiển.")
        return True

    
    def get_linear_velocities(self):
        """📊 Lấy vận tốc tuyến tính của hai bánh xe (m/s)"""
        rpmL, rpmR = self.get_rpm()
        
        if rpmL is None or rpmR is None:
            self.self_node.get_logger().error("❌ Không thể lấy RPM từ động cơ.")
            return None, None

        VL = self.rpm_to_linear(rpmL)
        VR = self.rpm_to_linear(-rpmR)

        return VL, VR

    
    def map(self, val, in_min, in_max, out_min, out_max):
        """🔄 Ánh xạ giá trị từ phạm vi `[in_min, in_max]` sang `[out_min, out_max]`"""
        if in_min == in_max:
            self.self_node.get_logger().error("❌ Lỗi! Không thể ánh xạ khi `in_min == in_max` (chia cho 0).")
            return None

        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    
    # def get_wheels_travelled(self):
    #     registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
    #     l_pul_hi = registers[0];l_pul_lo = registers[1]
    #     r_pul_hi = registers[2];r_pul_lo = registers[3]

    #     l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
    #     r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

    #     print(f"l_pulse: {l_pulse} === r_pulse: {r_pulse}")

    #     l_travelled = ((float(l_pulse)/self.cpr)*self.travel_in_one_rev) / self.R_Wheel  # unit in rad
    #     r_travelled = (float(r_pulse)/self.cpr)*self.travel_in_one_rev /  self.R_Wheel  # unit in rad

    #     return -l_travelled, r_travelled

    def get_wheels_travelled(self):
        """📏 Lấy quãng đường đã đi của bánh xe (đơn vị radian)"""
        
        registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        if registers is None or len(registers) < 4:
            self.self_node.get_logger().error("❌ Lỗi đọc dữ liệu vị trí bánh xe từ Modbus!")
            return None, None

        try:
            l_pul_hi, l_pul_lo = registers[0], registers[1]
            r_pul_hi, r_pul_lo = registers[2], registers[3]

            # Ghép thanh ghi thành giá trị 32-bit
            l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
            r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

            # Chuyển đổi sang góc quay (radian) 
            l_travelled = ((float(l_pulse) / self.cpr) * self.travel_in_one_rev) / self.R_Wheel
            r_travelled = ((float(r_pulse) / self.cpr) * self.travel_in_one_rev) / self.R_Wheel

            # 🔍 Debug mode: chỉ in ra khi cần thiết
            # if self.debug_mode:
            #     self.self_node.get_logger().info(f"📊 l_pulse: {l_pulse} === r_pulse: {r_pulse}")
            
            return -l_travelled, r_travelled

        except OverflowError:
            self.self_node.get_logger().error("❌ Lỗi tràn số khi tính toán vị trí bánh xe!")
            return None, None


    # def get_wheels_travelled(self):
    #     """🔍 Lấy quãng đường bánh xe đã đi (radian)"""
    #     registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
    #     if registers is None:
    #         return None, None

    #     l_pulse = (registers[0] << 16) | registers[1]
    #     r_pulse = (registers[2] << 16) | registers[3]

    #     l_travelled = (l_pulse / self.cpr) * self.travel_in_one_rev / self.R_Wheel
    #     r_travelled = (r_pulse / self.cpr) * self.travel_in_one_rev / self.R_Wheel

    #     return -l_travelled, r_travelled

    

    
    # def get_wheels_tick(self):
      
    #     try:
    #         registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        
    #         l_pul_hi = registers[0]; l_pul_lo = registers[1] 
    #         r_pul_hi = registers[2]; r_pul_lo = registers[3]

    #         l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
    #         r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

    #         return l_tick, r_tick
    #     except OverflowError:
    #         print("get_wheels_tick -> OverflowError occurred")

    def get_wheels_tick(self):
        """📊 Lấy số tick từ encoder của bánh xe trái và phải"""
        
        try:
            registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
            if registers is None or len(registers) < 4:
                self.self_node.get_logger().error("❌ Lỗi đọc dữ liệu encoder từ Modbus!")
                return None, None

            l_pul_hi, l_pul_lo = registers[0], registers[1]
            r_pul_hi, r_pul_lo = registers[2], registers[3]

            # Ghép thanh ghi thành giá trị 32-bit
            l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
            r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

            return l_tick, r_tick

        except OverflowError:
            self.self_node.get_logger().error("❌ Lỗi tràn số khi đọc encoder tick!")
            return None, None

        except IndexError:
            self.self_node.get_logger().error("❌ Dữ liệu Modbus trả về không đủ phần tử!")
            return None, None


    def set_position_async_control(self):
        """✅ Đặt chế độ điều khiển vị trí không đồng bộ"""
        try:
            result = self.client.write_register(self.POS_CONTROL_TYPE, self.ASYNC, unit=self.ID)
            self.self_node.get_logger().info("✅ Đã đặt chế độ ASYNC cho động cơ.")
            return result
        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi đặt chế độ ASYNC: {str(e)}")
            return None


    def move_left_wheel(self):
        """✅ Gửi lệnh di chuyển bánh xe trái"""
        try:
            result = self.client.write_register(self.CONTROL_REG, self.POS_L_START, unit=self.ID)
            self.self_node.get_logger().info("✅ Đã gửi lệnh di chuyển bánh xe trái.")
            return result
        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi gửi lệnh di chuyển bánh xe trái: {str(e)}")
            return None

    



    def close_connect(self):
        """✅ Đóng kết nối Modbus"""
        try:
            if self.client:
                self.client.close()
                self.self_node.get_logger().info("✅ Đã đóng kết nối Modbus.")
            else:
                self.self_node.get_logger().warn("⚠ Không có kết nối Modbus để đóng.")
        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi đóng kết nối Modbus: {str(e)}")

    
    # def is_motor_enabled(self):
    #     """🔍 Kiểm tra xem động cơ có đang ở trạng thái `enable` hay không"""
    #     registers = self.modbus_fail_read_handler(self.CONTROL_REG, 1)
    #     return registers is not None and registers[0] == self.ENABLE
    
    def is_motor_enabled(self):
        """✅ Kiểm tra xem động cơ có đang ở trạng thái `enable` hay không"""
        try:
            registers = self.modbus_fail_read_handler(self.CONTROL_REG, 1)
            if registers is None:
                self.self_node.get_logger().warn("⚠ Không thể kiểm tra trạng thái động cơ!")
                return False
            return registers[0] == self.ENABLE
        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi kiểm tra trạng thái động cơ: {str(e)}")
            return False


    def get_motor_status(self):
        """📊 Lấy tất cả thông tin về động cơ (có xử lý lỗi)"""
        try:
            return {
                "connected": self.is_connected(),
                "enabled": self.is_motor_enabled(),
                "mode": self.get_mode() if self.is_connected() else None,
                "rpm": self.get_rpm() if self.is_connected() else (None, None),
                "linear_velocity": self.get_linear_velocities() if self.is_connected() else (None, None),
                "position": self.get_wheels_travelled() if self.is_connected() else (None, None),
                "fault": self.get_fault_code() if self.is_connected() else (None, None),
            }
        except Exception as e:
            self.self_node.get_logger().error(f"❌ Lỗi khi lấy trạng thái động cơ: {str(e)}")
            return None
        
    def get_battery_voltage(self):
        """🔋 Đọc điện áp bus (battery voltage) từ driver"""
        try:
            registers = self.client.read_holding_registers(0x20A1, 1, unit=self.ID)
            if registers.isError():
                print("❌ Lỗi khi đọc điện áp bus!")
                return None
            voltage = registers.registers[0] * 0.01  # Quy đổi về đơn vị V
            print(f"🔋 Điện áp bus: {voltage}V")
            return voltage
        except Exception as e:
            print(f"❌ Lỗi khi đọc điện áp bus: {str(e)}")
            return None

    # def get_mode(self):
    #     registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)

#    def get_mode(self):
#         """
#         (Lấy chế độ điều khiển hiện tại)
#         """
#         registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)
#         mode = registers[0]
#         return mode


    def get_mode(self):
        """📌 Lấy chế độ điều khiển hiện tại"""
        try:
            registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)
            return registers[0] if registers else None
        except Exception as e:
            print(f"❌ Lỗi khi đọc chế độ điều khiển: {e}")
            return None
        
    def get_brake_state(self):
        """🛑 Kiểm tra trạng thái phanh"""
        try:
            result = self.client.read_holding_registers(0x201A, 1, unit=self.ID)
            if result and not result.isError():
                return "Closed" if result.registers[0] == 1 else "Open"
        except Exception as e:
            print(f"❌ Lỗi khi đọc trạng thái phanh: {str(e)}")
        return None

        
    """
    Hai hàm này dùng để kiểm tra lỗi của động cơ trái/phải bằng cách đọc dữ liệu từ self.L_FAULT (0x20A5) và self.R_FAULT (0x20A6).
    Nếu động cơ gặp lỗi, nó sẽ trả về mã lỗi tương ứng.

    Các lỗi có thể gặp:
        0x0001 - Over Voltage (Quá áp)
        0x0002 - Under Voltage (Dưới áp)
        0x0004 - Over Current (Quá dòng)
        0x0008 - Over Load (Quá tải)
        0x0010 - Current Out of Tolerance (Dòng điện không ổn định)
        0x0020 - Encoder Error (Lỗi encoder)
        0x0040 - Motor Bad (Lỗi động cơ)
        0x0080 - Reference Voltage Error (Lỗi điện áp tham chiếu)
        0x0100 - EEPROM Error (Lỗi EEPROM)
        0x0200 - Hall Sensor Error (Lỗi cảm biến Hall)
        0x0400 - Motor Over Temperature (Quá nhiệt)

    """

    def troubleshooting_left(self):
        """🔍 Kiểm tra lỗi động cơ trái"""
        try:
            registers = self.modbus_fail_read_handler(self.L_FAULT, 12)
            if registers:
                return registers
            else:
                return None
        except Exception as e:
            print(f"❌ Lỗi khi kiểm tra động cơ trái: {e}")
            return None


    def troubleshooting_right(self):
        """🔍 Kiểm tra lỗi động cơ phải"""
        try:
            registers = self.modbus_fail_read_handler(self.R_FAULT, 12)
            if registers:
                return registers
            else:
                return None
        except Exception as e:
            print(f"❌ Lỗi khi kiểm tra động cơ phải: {e}")
            return None


    def get_motor_faults(self):
        """🚨 Lấy mã lỗi động cơ trái/phải"""
        try:
            result = self.client.read_holding_registers(0x20A5, 2, unit=self.ID)
            if result and not result.isError():
                L_fault = result.registers[0]
                R_fault = result.registers[1]
                return L_fault, R_fault
        except Exception as e:
            print(f"❌ Lỗi khi đọc trạng thái lỗi: {str(e)}")
        return None, None
    
    def get_motor_temperature(self):
        """🌡️ Đọc nhiệt độ động cơ"""
        try:
            result = self.client.read_holding_registers(0x20A4, 1, unit=self.ID)
            if result and not result.isError():
                temp = result.registers[0]  # Đơn vị 1°C
                return temp
        except Exception as e:
            print(f"❌ Lỗi khi đọc nhiệt độ động cơ: {str(e)}")
        return None

    def get_driver_temperature(self):
        """🌡️ Đọc nhiệt độ driver"""
        try:
            result = self.client.read_holding_registers(0x20B0, 1, unit=self.ID)
            if result and not result.isError():
                temp = result.registers[0] * 0.1  # Đơn vị 0.1°C
                return temp
        except Exception as e:
            print(f"❌ Lỗi khi đọc nhiệt độ driver: {str(e)}")
        return None