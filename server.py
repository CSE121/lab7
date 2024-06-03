from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.request

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/location':
            self.handle_location_request()
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

def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=8000):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f'Starting httpd server on port {port}...')
    httpd.serve_forever()

if __name__ == "__main__":
    run()
