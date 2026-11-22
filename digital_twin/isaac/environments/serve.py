"""Serve one explicitly selected variant and the existing read-only display feed."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path


class Handler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        clean = path.split('?',1)[0]
        if clean == '/':
            return str(Path(self.directory)/'selector.html')
        if clean.startswith('/slim/'):
            # Serve the existing slim export through this one browser endpoint,
            # even when its separate HTTP process has not survived a VM reboot.
            session=json.loads(self.server.upstream.read_text())
            relative=Path(super().translate_path(clean[5:])).relative_to(self.directory)
            return str(Path(session.get('http_root',session['output']))/relative)
        # These remain owned and produced by the existing camera/twin services.
        if clean in ('/brick_observations.json','/brick_overlay.jpg','/brick_camera.jpg'):
            session = json.loads(self.server.upstream.read_text())
            return str(Path(session.get('http_root',session['output']))/clean[1:])
        return super().translate_path(path)

    def end_headers(self):
        self.send_header('Cache-Control','no-store')
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--upstream',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8767)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1',args.port),partial(Handler,directory=str(args.directory)))
    server.upstream = args.upstream
    server.serve_forever()


if __name__ == '__main__':
    main()
