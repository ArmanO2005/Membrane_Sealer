import serial.tools.list_ports

def find_port_by_serial(serial_number):
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.serial_number == serial_number:
            return p.device
    raise RuntimeError(f"Device with serial {serial_number} not found")

for p in serial.tools.list_ports.comports():
    print("device        :", p.device)
    print("description   :", p.description)
    print("serial_number :", p.serial_number)
    print("manufacturer  :", p.manufacturer)
    print("product       :", p.product)
    print("vid           :", p.vid)
    print("pid           :", p.pid)
    print("location      :", p.location)
    print("hwid          :", p.hwid)
    print("-" * 50)