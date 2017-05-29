/* global ROSLIB:false, malarkey:false */
(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .constant('_', window._)
    .constant('ROSLIB', ROSLIB)
    .constant('malarkey', malarkey)

})();
