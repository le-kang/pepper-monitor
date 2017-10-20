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
    vm.contentID = null;
    vm.messages = [];
    vm.message = {}; // current message
    vm.timer = null;
    vm.publish = publish;
    resetMessage();

    ros.connect('ws://198.18.0.1:9090');

    $rootScope.$on('pepper-message', function(event, content) {
      // validate content
      if (!_.isObjectLike(content)
        || !_.has(content, 'id')
        || !_.has(content, 'messages')
        || !_.isArray(content['messages']))
        return;
      // check if currently displaying an old content
      if (vm.contentID && vm.messages) {
        ros.publish('status', {
          id: vm.contentID,
          status: 'finished'
        });
        resetMessage();
      }
      if (vm.timer) $timeout.cancel(vm.timer);
      // assign new content
      vm.contentID = content['id'];
      vm.messages = content['messages'];
      displayNextMessage()
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
        id: vm.contentID,
        name: messageName,
        value: value
      });
      if (vm.timer) $timeout.cancel(vm.timer);
      $timeout(displayNextMessage, 0);
    }

    function displayNextMessage() {
      resetMessage();
      $rootScope.$apply();
      var next = vm.messages.shift();
      if (!next) {
        $timeout(function() {
          ros.publish('status', {
            id: vm.contentID,
            status: 'finished'
          });
        }, 1000);
        return;
      }
      _.assign(vm.message, next);
      ros.publish('status', {
        id: vm.contentID,
        status: 'displaying',
        current: vm.message.name
      });
      $rootScope.$apply();
      var timeout = (vm.message.timeout || DEFAULT_TIMEOUT) * 1000;
      vm.timer = $timeout(function() {
        displayNextMessage();
      }, timeout)
    }

    function resetMessage() {
      vm.message.name = null;
      vm.message.layout = 'column';
      vm.message.animation = true;
      vm.message.media = null;
      vm.message.dialog = null;
      vm.inputValue = null;
    }
  }

})();