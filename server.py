from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.request
import urllib.parse

# Global variables to store data
location_data = {}

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/location':
            self.handle_location_request()
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")

    def do_POST(self):
        if self.path == '/store_data':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            self.handle_store_data(post_data)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")

    def handle_location_request(self):
        with urllib.request.urlopen('http://wttr.in?format=%l\n') as response:
            data = response.read().decode('utf-8').strip()
            location = data.split(',')[0].replace(' ', '+')
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(location.encode('utf-8'))

    def handle_store_data(self, post_data):
        try:
            # Parse the URL-encoded form data
            data = urllib.parse.parse_qs(post_data.decode('utf-8'))
            global location_data
            location_data = {k: v[0] for k, v in data.items()}
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"Data stored successfully")
            
            # Print the stored data
            print("Stored data:")
            for key, value in location_data.items():
                print(f"{key}: {value}")
        except Exception as e:
            self.send_response(400)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"Error: {str(e)}".encode('utf-8'))

def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=1234):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f'Starting httpd server on port {port}...')
    httpd.serve_forever()

if __name__ == "__main__":
    run()
