(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .directive('typewriter', typewriter);

  function typewriter($rootScope, malarkey, $timeout, $interval) {
    var directive = {
      restrict: 'A',
      scope: {
        value: '@',
        speed: '@'
      },
      link: link
    };

    return directive;

    function link(scope, el) {
      el.addClass('typewriter');
      var typewriter = malarkey(el[0], { typeSpeed: scope.speed });
      // wait for animation finish
      $timeout(function() {
        typewriter.type(scope.value);
      }, 1000);
      // show buttons after text typing finished
      var checkTypingStatus = $interval(function() {
        if (!typewriter.isRunning()) {
          $rootScope.$emit('show-buttons');
          $interval.cancel(checkTypingStatus)
        }
      }, 200)
    }
  }

})();