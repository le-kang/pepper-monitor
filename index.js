// Simple http server for local development and testing

var path = require('path');
var express = require('express');

var app = express();

app.use(express.static(path.join(__dirname, '/.tmp/serve')));
app.use(express.static(path.join(__dirname, '/app')));

app.listen(3000);