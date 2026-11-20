"""Loopback-only review server; JSON gzip for the generated operational run."""
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import argparse,os
class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control','no-store')
        super().end_headers()
    def do_GET(self):
        if self.path.split('?')[0]=='/run.json' and 'gzip' in self.headers.get('Accept-Encoding','') and Path('run.json.gz').exists():
            data=Path('run.json.gz').read_bytes()
            self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Encoding','gzip');self.send_header('Vary','Accept-Encoding');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        else:super().do_GET()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8745);a=p.parse_args();os.chdir(Path(__file__).resolve().parent);ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
