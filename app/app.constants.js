/* global ROSLIB:false, malarkey:false, katex:false */
(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .constant('_', window._)
    .constant('ROSLIB', ROSLIB)
    .constant('malarkey', malarkey)
    .constant('katex', katex)

})();
