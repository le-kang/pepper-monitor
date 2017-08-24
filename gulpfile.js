'use strict';

var gulp = require('gulp');
var wiredep = require('wiredep').stream;
var saveLicense = require('uglify-save-license');
var mainBowerFiles = require('main-bower-files');
var del = require('del');
//////////////////////////////////////////////////////////
// Gulp Plugins
//////////////////////////////////////////////////////////
var angularFilesort = require('gulp-angular-filesort');
var angularTemplatecache = require('gulp-angular-templatecache');
var autoprefixer = require('gulp-autoprefixer');
var cleanCSS = require('gulp-clean-css');
var eslint = require('gulp-eslint');
var filter = require('gulp-filter');
var flatten = require('gulp-flatten');
var htmlmin = require('gulp-htmlmin');
var gulpif = require('gulp-if');
var inject = require('gulp-inject');
var ngAnnotate = require('gulp-ng-annotate');
var nodemon = require('gulp-nodemon');
var rename = require('gulp-rename');
var replace = require('gulp-replace');
var rev = require('gulp-rev');
var revReplace = require('gulp-rev-replace');
var sass = require('gulp-sass');
var size = require('gulp-size');
var uglify = require('gulp-uglify');
var useref = require('gulp-useref');
var gutil = require('gulp-util');
var run = require('gulp-run');
//////////////////////////////////////////////////////////



/**
 * Scripts Task
 */
gulp.task('scripts',  function() {
  return buildScripts();
});

/**
 * Styles Task
 */
gulp.task('styles', function() {
  return buildStyles();
});

gulp.task('katex-fonts', function() {
  return gulp.src('app/bower_components/katex/dist/fonts/**/*')
    .pipe(gulp.dest('dist/styles/fonts'));
});

gulp.task('katex-images', function() {
  return gulp.src('app/bower_components/katex/dist/images/**/*')
    .pipe(gulp.dest('dist/styles/images'));
});

/**
 * Inject Task
 */
gulp.task('inject', ['scripts', 'styles'], function() {
  var injectStyles = gulp.src('.tmp/serve/app/**/*.css', { read: false });

  var injectScripts = gulp.src([
    'app/**/*.module.js',
    'app/**/*.js',
    '!app/**/*.spec.js',
    '!app/**/*.mock.js',
    '!app/bower_components/**/*'
  ])
    .pipe(angularFilesort()).on('error', errorHandler('AngularFilesort'));

  var injectOptions = {
    ignorePath: ['app', '.tmp/serve'],
    addRootSlash: false
  };

  return gulp.src('app/*.html')
    .pipe(inject(injectStyles, injectOptions))
    .pipe(inject(injectScripts, injectOptions))
    .pipe(wiredep())
    .pipe(gulp.dest('.tmp/serve/'));
});

/**
 * Templates Task
 */
gulp.task('templates', function() {
  return gulp.src('app/views/*.html')
    .pipe(htmlmin({ collapseWhitespace: true }))
    .pipe(angularTemplatecache('templates.js', {
      module: 'pepperMonitor',
      root: 'views'
    }))
    .pipe(gulp.dest('.tmp/serve/app'));
});

/**
 * HTML Task
 */
gulp.task('html', ['inject', 'templates'], function() {
  var injectTemplates = gulp.src('.tmp/serve/app/templates.js', { read: false });
  var injectOptions = {
    starttag: '<!-- inject: templates -->',
    ignorePath: '.tmp/serve',
    addRootSlash: false
  };

  return gulp.src('.tmp/serve/*.html')
    .pipe(inject(injectTemplates, injectOptions))
    .pipe(useref())
    .pipe(gulpif('**/*.{js,css}', rev()))
    .pipe(gulpif('**/*.js', ngAnnotate()))
    .pipe(gulpif('**/*.js', uglify({ preserveComments: saveLicense }))).on('error', errorHandler('Uglify'))
    .pipe(gulpif('**/*.css', cleanCSS({ processImport: false })))
    .pipe(revReplace())
    .pipe(gulpif('*.html', htmlmin({ collapseWhitespace: true })))
    .pipe(gulp.dest('dist/'))
    .pipe(size({ title: 'dist/', showFiles: true }));
});

/**
 * Fonts Task
 */
gulp.task('fonts', function() {
  return gulp.src(mainBowerFiles())
    .pipe(filter('**/*.{oft,eot,svg,ttf,woff,woff2}'))
    .pipe(flatten())
    .pipe(gulp.dest('dist/fonts/'));
});

/**
 * Other Task
 */
gulp.task('other', function() {
  var fileFilter = filter(function(file) {
    return file.stat.isFile();
  });

  return gulp.src(['app/**/*', '!app/bower_components/**/*', '!**/*.{html,css,js,scss}', '*.sh', '*.py'])
    .pipe(fileFilter)
    .pipe(gulp.dest('dist/'))
});

/**
 * Watch Task
 */
gulp.task('watch', ['inject'], function() {
  gulp.watch(['app/**/*.html', 'app/**/*.js', 'app/**/*.scss', 'bower.json'], ['inject']);
});

/**
 * Serve Task
 */
gulp.task('serve', ['watch'], function() {
  gulp.start('nodemon');
});

/**
 * Build Task
 */
gulp.task('build', ['html', 'fonts', 'other', 'katex-fonts', 'katex-images']);

/**
 * Clean Task
 */
gulp.task('clean', function() {
  return del(['.tmp/', 'dist/']);
});


/**
 * Node Monitor Task
 */
gulp.task('nodemon', function() {
  return nodemon({
    script: 'index.js',
    watch: ['index.js']
  });
});

/**
 * Default Task
 */
gulp.task('default', ['clean'], function() {
  gulp.start('build');
});



/**
 * Reusable Functions
 */
function errorHandler(title) {
  return function(err) {
    gutil.log(gutil.colors.red('[' + title + ']'), err.toString());
    this.emit('end');
  };
}

function buildScripts() {
  return gulp.src(['app/**/*.js', '!app/bower_components/**/*', '!app/assets/lib/**/*'])
    .pipe(eslint())
    .pipe(eslint.format())
    .pipe(size());
}

function buildStyles() {
  var injectFiles = gulp.src('app/styles/*.scss', { read: false });

  var injectOptions = {
    transform: function(filePath) {
      filePath = filePath.replace('app/', '');
      return '@import "' + filePath + '";';
    },
    starttag: '// injector',
    endtag: '// endinjector',
    addRootSlash: false
  };

  return gulp.src('app/app.scss')
    .pipe(inject(injectFiles, injectOptions))
    .pipe(wiredep())
    .pipe(sass({ outputStyle: 'expanded' })).on('error', errorHandler('Sass'))
    .pipe(autoprefixer()).on('error', errorHandler('Autoprefixer'))
    .pipe(gulp.dest('.tmp/serve/app'));
}
