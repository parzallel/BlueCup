import serial
from colorama import Fore, Style, init
from math import radians, degrees, sin, cos, atan2, sqrt
import time

init(autoreset=True)

class Kalman1D:
    def __init__(self, process_variance, measurement_variance):
        self.x = None
        self.P = 1.0
        self.Q = process_variance
        self.R = measurement_variance
    def input_latest_noisy_measurement(self, measurement):
        if self.x is None:
            self.x = measurement
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P
        return self.x

def nmea_to_decimal(coord, direction):
    if not coord or coord == '' or float(coord) == 0:
        return None
    deg = int(float(coord) // 100)
    minutes = float(coord) - deg * 100
    decimal = deg + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi/2)**2 + cos(phi1)*cos(phi2)*sin(dlambda/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_lon = radians(lon2 - lon1)
    x = sin(delta_lon) * cos(phi2)
    y = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(delta_lon)
    bearing = atan2(x, y)
    bearing = degrees(bearing)
    return (bearing + 360) % 360

def smallest_angle_diff(target, current):
    diff = (target - current + 180) % 360 - 180
    return diff

def find_usb_serial_port():
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.description or 'Serial' in port.description:
            return port.device
    return None

def moving_average(window, new_value, size=5):
    window.append(new_value)
    if len(window) > size:
        window.pop(0)
    return sum(window) / len(window)

def input_targets():
    n = int(input("تعداد نقاط هدف: "))
    targets = []
    for i in range(n):
        lat = float(input(f"عرض جغرافیایی هدف شماره {i+1}: ").strip())
        lon = float(input(f"طول جغرافیایی هدف شماره {i+1}: ").strip())
        targets.append((lat, lon))
    return targets

def print_separator():
    print("\n"+"#"*50+"\n")

def main():
    try:
        # --- دریافت چندین نقطه هدف ---
        targets = input_targets()
    except Exception:
        print("⚠️  لطفاً مختصات را به صورت عددی وارد کنید.")
        return

    port = find_usb_serial_port()
    if not port:
        print("درگاه سریال پیدا نشد.")
        return

    print(f"درگاه متصل شده: {port}")

    columns = [
        "عرض جغرافیایی", "طول جغرافیایی",
        "اختلاف زاویه", "وضعیت هدف", "فاصله تا هدف"
    ]
    all_data = []      # ذخیره تمام داده‌ها برای همه اهداف

    # --- تعریف فیلتر کالمن ---
    kalman_lat = Kalman1D(process_variance=1e-6, measurement_variance=1e-4)
    kalman_lon = Kalman1D(process_variance=1e-6, measurement_variance=1e-4)

    # --- میانگین متحرک ---
    WINDOW_SIZE = 5
    lat_window = []
    lon_window = []

    try:
        with serial.Serial(port, baudrate=115200, timeout=0.1) as ser:
            print("در حال دریافت داده GPS ... (Ctrl+C بزنید برای خروج)")
            for target_idx, target in enumerate(targets):
                ref_lat, ref_lon = target
                print_separator()
                print(f"🎯 هدف {target_idx+1}  --->  عرض: {ref_lat:.6f}  |  طول: {ref_lon:.6f}")
                print_separator()
                # df = pd.DataFrame(columns=columns)
                last_points = []
                latest_result = None
                last_update = time.time()
                while True:
                    now = time.time()
                    line = ser.readline()
                    if line:
                        try:
                            decoded = line.decode("ascii", "replace").strip()
                            if decoded.startswith('$GPGGA'):
                                parts = decoded.split(',')
                                if len(parts) >= 6:
                                    lat = nmea_to_decimal(parts[2], parts[3])
                                    lon = nmea_to_decimal(parts[4], parts[5])
                                    if lat is not None and lon is not None:
                                        filt_lat = kalman_lat.input_latest_noisy_measurement(lat)
                                        filt_lon = kalman_lon.input_latest_noisy_measurement(lon)
                                        ma_lat = moving_average(lat_window, filt_lat, WINDOW_SIZE)
                                        ma_lon = moving_average(lon_window, filt_lon, WINDOW_SIZE)
                                        latest_result = (ma_lat, ma_lon)
                        except Exception:
                            continue

                    if latest_result and now - last_update >= 0.1:
                        ma_lat, ma_lon = latest_result
                        last_points.append((ma_lat, ma_lon))
                        if len(last_points) > 3:
                            last_points.pop(0)
                        angle_diff, status, dist = None, None, None
                        disp_diff, disp_dist, msg = "-", "-", ""
                        color = Style.RESET_ALL
                        if len(last_points) == 3:
                            dir_to_target = calculate_bearing(
                                last_points[2][0], last_points[2][1],
                                ref_lat, ref_lon
                            )
                            move_bearing = calculate_bearing(
                                last_points[1][0], last_points[1][1],
                                last_points[2][0], last_points[2][1]
                            )
                            angle_diff = smallest_angle_diff(dir_to_target, move_bearing)
                            dist = haversine(
                                last_points[2][0], last_points[2][1],
                                ref_lat, ref_lon
                            )
                            status = 1 if dist < 2.5 else 0
                            disp_diff = f"{angle_diff:+.2f}"
                            disp_dist = f"{dist:.2f}"
                            if abs(angle_diff) < 1:
                                color = Fore.GREEN
                                msg = " - کاملا روی هدف"
                            elif abs(angle_diff) < 10:
                                color = Fore.RED
                                msg = " - تقریباً نزدیک جهت"
                        else:
                            disp_dist = "-"
                            status = "-"

                        print(f"{color}عرض: {ma_lat:.6f} | طول: {ma_lon:.6f}"
                              f" | اختلاف زاویه: {disp_diff:>7}° | فاصله تا هدف: {Fore.YELLOW}{disp_dist:>7} متر{color}"
                              f" | وضعیت هدف: {status}{msg}{Style.RESET_ALL}")

                        row = {
                            "عرض جغرافیایی": ma_lat,
                            "طول جغرافیایی": ma_lon,
                            "اختلاف زاویه": angle_diff if angle_diff is not None else "",
                            "وضعیت هدف": status if status is not None else "",
                            "فاصله تا هدف": dist if dist is not None else ""
                        }
                        # df.loc[len(df)] = row
                        last_update = now

                        if status == 1:    # وقتی به هدف رسیدی، سوییچ کن
                            print(f"{Fore.CYAN}✅ به شعاع هدف {target_idx+1} رسیدی! آماده حرکت به نقطه بعدی ...{Style.RESET_ALL}")
                            break

                # افزودن داده‌ها و خطوط فاصله برای خروجی نهایی
                # all_data.extend(df.values.tolist())
                all_data.extend([["-"]*5]*3)  # سه خط فاصله

    except KeyboardInterrupt:
        print("\n⏹️  دریافت داده متوقف شد. خروجی ذخیره می‌شود...")



if __name__ == "__main__":
    main()
