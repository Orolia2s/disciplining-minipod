import os
from conan import ConanFile
from conan.tools.gnu import Autotools, AutotoolsToolchain, AutotoolsDeps
from conan.tools.scm import Git
from conan.tools.files import save, load, copy

class DiscipliningMinipodConan(ConanFile):
    name = 'oscillator-disciplining'
    author = 'M. Lorentz <matthias.lorentz@orolia2s.com>'
    url = 'https://github.com/Orolia2s/disciplining-minipod.git'
    license = 'LGPL-2.1'
    description = 'disciplining algorithm for Atomic Reference Time Card'
    topics = ('timing')

    settings = ('os', 'arch', 'compiler', 'build_type')
    options = {
        'shared': [True, False],
        'fPIC': [True, False]
    }
    default_options = {
        'shared': False,
        'fPIC': True
    }

    exports_sources = 'Makefile', 'src/*.[ch]', 'include/*.h', 'README.md'

    def _get_latest_tag(self):
        git = Git(self, folder=self.recipe_folder)
        return git.run('tag --sort "-version:refname" --merged').split('\n', 1)[0]

    def export(self):
        # Only files that are necessary for the evaluation of the conanfile.py
        # recipe must be exported with this method. Files necessary for
        # building from sources should be exported with the exports_sources
        # attribute or the export_source() method.
        save(self, os.path.join(self.export_folder, 'version.txt'), self._get_latest_tag())

    def set_version(self):
        try:
            self.version = load(self, 'version.txt')
        except:
            self.version = self._get_latest_tag()

    def configure(self):
        self.settings.rm_safe('compiler.libcxx')
        self.settings.rm_safe('compiler.cppstd')
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def generate(self):
        autotools = AutotoolsDeps(self)
        autotools.environment.define('Version', self.version)
        autotools.generate()
        AutotoolsToolchain(self).generate()

    def build(self):
        autotools = Autotools(self)
        autotools.make('shared' if self.options.shared else 'static')
        autotools.make('clean')

    def package(self):
        copy(self, '*.h',
             os.path.join(self.source_folder, 'include'),
             os.path.join(self.package_folder, 'include'))
        copy(self, '*.a', self.build_folder,
             os.path.join(self.package_folder, 'lib'))
        copy(self, '*.so', self.build_folder,
             os.path.join(self.package_folder, 'lib'))
        copy(self, 'version.txt', self.source_folder,
             os.path.join(self.package_folder, 'lib'))
        copy(self, 'README.md', self.source_folder,
             os.path.join(self.package_folder, 'lib'))

    def package_info(self):
        self.cpp_info.libs = ['oscillator-disciplining']
        self.cpp_info.system_libs = ['m']
