import threading
import json
import time
from datetime import datetime
import pandas as pd
import os
import argparse

import serial
from flask import Flask
import dash
from dash import dcc, html, Output, Input

# ---- CONFIGURATIONS ----
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 1150200
EXCEL_FILE = 'sensor_data_log.xlsx'
PORT = 8050  # Default port for the Flask server
# -----------------------------------------

# Shared data structures
sensor_data = {}
logs = []
excel_data = []  # Store all sensor readings for Excel export
package_buffer = {}  # Buffer to collect all sensors from current package
data_lock = threading.Lock()  # Thread-safe access to excel_data
start_time = None  # Track when the system started

# Start the Flask server
server = Flask(__name__)
app = dash.Dash(__name__, server=server)

def parse_arguments():
    """Parse command line arguments for configuration."""
    parser = argparse.ArgumentParser(description='Sensor Dashboard')
    parser.add_argument('--serial', default='/dev/ttyACM0', help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=1150200, help='Baud rate (default: 1150200)')
    parser.add_argument('--port', type=int, default=8050, help='Web server port (default: 8050)')
    parser.add_argument('--excel', default='sensor_data_log.xlsx', help='Excel file name (default: sensor_data_log.xlsx)')
    
    return parser.parse_args()

args = parse_arguments()
SERIAL_PORT = args.serial
BAUD_RATE = args.baud
EXCEL_FILE = args.excel
PORT = args.port

print(f"|{SERIAL_PORT}|")

def save_to_excel():
    """Save accumulated sensor data to Excel file."""
    if not excel_data:
        return
    
    with data_lock:
        df = pd.DataFrame(excel_data)
        
        # Create Excel file with multiple sheets if it doesn't exist
        if not os.path.exists(EXCEL_FILE):
            with pd.ExcelWriter(EXCEL_FILE, engine='openpyxl') as writer:
                df.to_excel(writer, sheet_name='All_Data', index=False)
        else:
            # Append to existing file
            with pd.ExcelWriter(EXCEL_FILE, engine='openpyxl', mode='a', if_sheet_exists='overlay') as writer:
                # Load existing data and append new data
                try:
                    existing_df = pd.read_excel(EXCEL_FILE, sheet_name='All_Data')
                    combined_df = pd.concat([existing_df, df], ignore_index=True)
                except:
                    combined_df = df
                
                combined_df.to_excel(writer, sheet_name='All_Data', index=False)
        
        # Clear the buffer after saving
        excel_data.clear()
        print(f"Data saved to {EXCEL_FILE}")
        
def flatten_sensor_data(data, prefix=""):
    """Recursively flatten nested dictionaries into separate columns."""
    flattened = {}
    for key, value in data.items():
        new_key = f"{prefix}_{key}" if prefix else key
        if isinstance(value, dict):
            # Recursively flatten nested dictionaries
            flattened.update(flatten_sensor_data(value, new_key))
        else:
            flattened[new_key] = value
    return flattened

def serial_reader():
    """Background thread: read lines from serial, parse JSON, update sensor_data & logs."""
    global start_time
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
        return

    save_counter = 0
    start_time = datetime.now()  # Initialize start time when serial reader begins
    print(f"System started at: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            
            payload = json.loads(line)
            current_time = datetime.now()
            
            # Calculate relative time since system start in seconds
            relative_time = (current_time - start_time).total_seconds()
            
            # Initialize package row with relative timestamp
            package_row = {
                'time_seconds': round(relative_time, 3),
                'absolute_timestamp': current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            }
            
            # Process all entries in the current package
            for entry in payload:
                t = entry.get('type')
                content = entry.get('content', {})
                source = content.get('source', 'UNKNOWN')

                if t == 'SENSOR_DATA':
                    # store the entire sensorData dict plus timestamp
                    sd = content.get('sensorData', {})
                    sensor_data[source] = {
                        'values': sd,
                        'ts': current_time
                    }
                    
                    flatten_sensor_data_value = flatten_sensor_data(sd, source)
                    package_row.update(flatten_sensor_data_value)

                elif t in ('INFO','WARNING','ERROR'):
                    msg = content.get('message', '')
                    logs.append({
                        'ts': current_time,
                        'relative_time': relative_time,
                        'type': t,
                        'source': source,
                        'message': msg
                    })
            
            # Add the complete package row to excel_data
            if any(key.startswith(('BNO055_', 'BAR1_', 'BAR2_', 'LIS3DHTR_', 'GPS_', 'MainActuators_', 'DrogueActuators_', 'Buzzer_', 'Voltage_','Termoresistenze_')) for key in package_row.keys()):
                with data_lock:
                    excel_data.append(package_row)

            # trim logs to last 200 messages
            if len(logs) > 200:
                del logs[:-200]
            
            # Save to Excel every 100 readings or every 30 seconds
            save_counter += 1
            if save_counter >= 100:
                save_to_excel()
                save_counter = 0
                
            print(f"Valid JSON received at T+{relative_time:.3f}s")
            
        except json.JSONDecodeError:
            # skip invalid JSON
            print(f"Invalid JSON received!")
            continue
        except Exception as e:
            print(f"Error in serial_reader: {e}")
            time.sleep(1)
            
def periodic_save():
    """Periodically save data to Excel every 5 seconds."""
    while True:
        time.sleep(5)
        save_to_excel()

# Start background threads
threading.Thread(target=serial_reader, daemon=True).start()
threading.Thread(target=periodic_save, daemon=True).start()

# Dash layout
app.layout = html.Div(style={'display': 'flex', 'height': '100vh', 'fontFamily': 'Arial'}, children=[

    # Left: Logs
    html.Div(style={
        'width': '35%', 'padding': '10px', 'borderRight': '1px solid #ccc', 'overflowY': 'scroll'
    }, children=[
        html.H3("System Logs"),
        html.Div(id='log-box', style={'whiteSpace': 'pre-wrap', 'fontFamily': 'Courier New'}),
        html.Hr(),
        html.P(f"Data logging to: {EXCEL_FILE}", style={'fontSize': '12px', 'color': 'gray'}),
        html.Div(id='excel-status', style={'fontSize': '12px', 'color': 'green'})
    ]),

    # Right: Sensor Data
    html.Div(style={'flex': '1', 'padding': '10px'}, children=[
        html.H3("Sensor Readings"),
        html.Div(id='sensor-panel')
    ]),

    # Interval to trigger updates every second
    dcc.Interval(id='interval-update', interval=1000, n_intervals=0)
])

@app.callback(
    Output('log-box', 'children'),
    Input('interval-update', 'n_intervals')
)
def update_logs(n):
    """Render the last logs in color-coded fashion."""
    lines = []
    for entry in logs[-50:]:  # show last 50
        relative_time = entry.get('relative_time', 0)
        typ = entry['type']
        src = entry['source']
        msg = entry['message']
        # color by type
        color = {
            'INFO': 'black',
            'WARNING': 'orange',
            'ERROR': 'red'
        }.get(typ, 'black')
        lines.append(
            html.Div(f"T+{relative_time:.3f}s [{src}] {typ}: {msg}", style={'color': color})
        )
    return lines

@app.callback(
    Output('excel-status', 'children'),
    Input('interval-update', 'n_intervals')
)
def update_excel_status(n):
    """Show Excel logging status."""
    with data_lock:
        buffer_size = len(excel_data)
    
    status_text = f"Excel buffer: {buffer_size} records"
    if os.path.exists(EXCEL_FILE):
        mod_time = datetime.fromtimestamp(os.path.getmtime(EXCEL_FILE))
        status_text += f" | Last saved: {mod_time.strftime('%H:%M:%S')}"
    
    return status_text

@app.callback(
    Output('sensor-panel', 'children'),
    Input('interval-update', 'n_intervals')
)
def update_sensors(n):
    """Render each sensor's latest readings and time since update."""
    now = datetime.now()
    panels = []
    
    # Expected sensors: BNO055, BAR1, BAR2, LIS3DHTR, GPS
    expected_sensors = ['BNO055', 'BAR1', 'BAR2', 'LIS3DHTR', 'GPS', 'MainActuators', 'DrogueActuators', 'Buzzer', 'Voltage', "Termoresistenze"]
    
    for src in expected_sensors:
        if src in sensor_data:
            data = sensor_data[src]
            vals = data['values']
            ts = data['ts']
            elapsed = (now - ts).total_seconds()
            
            # Build a table of key/value with sensor-specific formatting
            rows = []
            for k, v in vals.items():
                # Format values based on sensor type and parameter
                if isinstance(v, float):
                    if 'temp' in k.lower() or 'temperature' in k.lower():
                        formatted_v = f"{v:.2f}°C"
                    elif 'pressure' in k.lower():
                        formatted_v = f"{v:.2f} hPa"
                    elif 'altitude' in k.lower():
                        formatted_v = f"{v:.2f} m"
                    elif any(axis in k.lower() for axis in ['x', 'y', 'z']):
                        formatted_v = f"{v:.3f}"
                    else:
                        formatted_v = f"{v:.3f}"
                else:
                    formatted_v = str(v)
                
                rows.append(html.Tr([
                    html.Td(k, style={'fontWeight': 'bold', 'minWidth': '100px'}), 
                    html.Td(formatted_v)
                ]))
            
            # Color code based on data freshness
            border_color = '#4CAF50' if elapsed < 2 else '#FF9800' if elapsed < 5 else '#F44336'
            
            panels.append(html.Div(style={
                'border': f'2px solid {border_color}', 'borderRadius': '5px',
                'padding': '8px', 'marginBottom': '10px', 'backgroundColor': '#f9f9f9'
            }, children=[
                html.H4(src, style={'margin': '0 0 8px 0', 'color': border_color}),
                html.P(f"Last update: {elapsed:.1f}s ago", 
                      style={'fontStyle':'italic', 'fontSize':'90%', 'margin': '0 0 8px 0'}),
                html.Table(rows, style={'width':'100%', 'borderCollapse': 'collapse'})
            ]))
        else:
            # Show missing sensor
            panels.append(html.Div(style={
                'border': '2px solid #ccc', 'borderRadius': '5px',
                'padding': '8px', 'marginBottom': '10px', 'backgroundColor': '#f0f0f0'
            }, children=[
                html.H4(src, style={'margin': '0 0 8px 0', 'color': '#888'}),
                html.P("No data received", style={'fontStyle':'italic', 'color': '#888'})
            ]))
    
    return panels

if __name__ == '__main__':
    try:
        app.run(debug=False, port=PORT, host='0.0.0.0')
    finally:
        # Save any remaining data when shutting down
        save_to_excel()
        print("Dashboard shut down - final data saved to Excel")