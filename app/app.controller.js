(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .controller('AppController', AppController);

  /** @ngIngect */
  function AppController($rootScope, ros, $window, _, $timeout, $mdDialog, $mdToast) {
    var vm = this;
    var DEFAULT_TIMEOUT = 60;
    vm.renderJobs = [];
    vm.message = {};
    vm.publish = publish;
    clearMessage();

    ros.connect('ws://198.18.0.1:9090');

    $rootScope.$on('pepper-message', function(event, messages) {
      if (!_.isArray(messages)) return;
      // cancel any unfinished rendering job and empty job list
      _.forEach(vm.renderJobs, function(job) {
        $timeout.cancel(job);
      });
      vm.renderJobs = [];
      // schedule message display
      var job;
      var timeout = 0;
      while (messages.length) {
        var message = _.clone(messages.shift());
        job = createRenderJob(message, timeout);
        vm.renderJobs.push(job);
        timeout += (message.timeout || DEFAULT_TIMEOUT) * 1000;
      }
      // stop displaying last message
      job = $timeout(function() {
        clearMessage();
        $rootScope.$apply()
      }, timeout);
      vm.renderJobs.push(job);
    });

    $rootScope.$on('ros-disconnected', function() {
      var confirm = $mdDialog.confirm()
        .title('Failed to connect with ROS bridge')
        .textContent('Try to refresh to reconnect to ROS.')
        .ok('Refresh now')
        .cancel('Refresh later');
      $mdDialog.show(confirm).then(function() {
        $window.location.reload();
      }, function() {
        var toast = $mdToast.simple()
          .textContent('Not connected with ROS bridge.')
          .action('Refresh')
          .highlightAction(true)
          .hideDelay(false);
        $mdToast.show(toast).then(function() {
          $window.location.reload();
        });
      });
    });

    function publish(messageName, value) {
      if (value === '' || value === null) return;
      ros.publish('feedback', {
        name: messageName,
        value: value
      });
      clearMessage();
    }

    function clearMessage() {
      vm.message.name = null;
      vm.message.layout = 'column';
      vm.message.animation = true;
      vm.message.media = null;
      vm.message.dialog = null;
      vm.inputValue = null;
    }

    function createRenderJob(message, timeout) {
      return $timeout(function() {
        clearMessage();
        $rootScope.$apply();
        _.assign(vm.message, message);
        ros.publish('status', { status: message.name });
        $rootScope.$apply();
        vm.renderJobs.shift();
      }, timeout)
    }
  }

})();