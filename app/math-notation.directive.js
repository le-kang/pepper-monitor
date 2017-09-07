(function() {
  'use strict';

  angular
    .module('pepperMonitor')
    .directive('mathNotation', mathNotation);

  function mathNotation(katex) {
    var directive = {
      restrict: 'E',
      scope: {
        expression: '@'
      },
      link: link
    };

    return directive;

    function link(scope, el) {
      el.replaceWith(katex.renderToString(scope.expression))
    }
  }

})();