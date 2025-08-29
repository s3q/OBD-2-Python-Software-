def _parse_hex_response( response: str) :
    """Parse hex response into byte array"""
    try:
        # Remove spaces and common error responses
        clean_response = response.replace(' ', '').replace('\r', '').replace('\n', '')

        # Check for error responses
        error_responses = ['NODATA', 'ERROR', 'TIMEOUT', '?', 'UNABLETOCONNECT', 'BUSBUSY']
        if any(err in clean_response.upper() for err in error_responses):
            return []

        # Convert hex pairs to integers
        if len(clean_response) % 2 == 0:
            return [int(clean_response[i:i+2], 16) for i in range(0, len(clean_response), 2)]
        else:
            return []

    except Exception as e:
        self.logger.error(f"Failed to parse hex response: {response} - {e}")
        return []


i = "A111"
bit = _parse_hex_response(i)
print(bit)


l = [1, 2, 3, 4, 5, 2]
v = l[1:5]

print(l[0:3])

bits = "01000001000011000"
print(int(bits, 2))

def parse_pid_response(resp: str):
    """
    Assumes resp is already cleaned (e.g., '410C0FA0').
    """
    mode = resp[:2]   # '41'
    pid  = resp[2:4]  # '0C'
    data = resp[4:]   # '0FA0' (remaining hex string)
    return mode, pid, data

raw = "410C0000>>>"
command="010C"
if raw.startswith(command):
    responrawse = raw[len(command):].strip()

mode, pid, data = parse_pid_response(raw)

print("ECU respond --> ",mode, pid, data, " ")