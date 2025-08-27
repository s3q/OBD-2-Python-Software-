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