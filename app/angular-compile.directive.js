(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .directive('dynamicTemplate', dynamicTemplate);

  function dynamicTemplate($compile) {
    var directive = {
      restrict: 'A',
      scope: {
        monitor: '='
      },
      link: link
    };

    return directive;

    function link(scope, el) {
      el.html(scope.monitor.message.dialog.template);
      $compile(el.contents())(scope)
    }
  }

})();