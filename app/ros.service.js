(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .factory('ros', ros);

  /** @ngInject **/
  function ros(ROSLIB, $rootScope) {
    var ros = null;
    var listener = null;
    var statusPublisher = null;
    var feedbackPublisher = null;
    return {
      connect: function(address) {
        ros = new ROSLIB.Ros({
          url: address
        });
        ros.on('connection', function() {
          listener = new ROSLIB.Topic({
            ros: ros,
            name: '/pepper_monitor/message',
            messageType: 'std_msgs/String'
          });
          listener.subscribe(function(message) {
            try {
              var json = JSON.parse(message.data);
            } catch(e) {
              return;
            }
            $rootScope.$emit('pepper-message', json);
          });
          statusPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/pepper_monitor/status',
            messageType: 'std_msgs/String'
          });
          feedbackPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/pepper_monitor/feedback',
            messageType: 'std_msgs/String'
          });
        });
        ros.on('close', function() {
          ros = null;
          listener = null;
          statusPublisher = null;
          feedbackPublisher = null;
          $rootScope.$emit('ros-disconnected');
        });
      },
      publish: function(type, data) {
        if (type !== 'status' && type !== 'feedback') return;
        if (type === 'status' && !statusPublisher) return;
        if (type === 'feedback' && !feedbackPublisher) return;
        try {
          var jsonString = JSON.stringify(data);
        } catch(e) {
          return;
        }
        var publisher = type === 'status' ? statusPublisher : feedbackPublisher;
        var message = new ROSLIB.Message({
          data: jsonString
        });
        publisher.publish(message);
      }
    }
  }
})();
