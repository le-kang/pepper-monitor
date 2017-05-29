#!/usr/bin/python

import SimpleHTTPServer
import SocketServer
import os

HOST_NAME = "198.18.0.1"
PORT_NUMBER = 9000

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

httpd = SocketServer.TCPServer((HOST_NAME, PORT_NUMBER), Handler)

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    pass
httpd.server_close()
