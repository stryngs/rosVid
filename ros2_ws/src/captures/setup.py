from setuptools import setup

pkgName = 'captures'

setup(author = 'stryngs',
      author_email = '127.0.0.1@nowhere.com',
      data_files = [('share/ament_index/resource_index/packages', [f'resource/{pkgName}']),
                    (f'share/{pkgName}', ['package.xml'])],
      description = 'Transport and viewing mechanisms for video streams.',
      entry_points = {'console_scripts': [f'phys = {pkgName}.phys:main',
                                          f'rtsp = {pkgName}.rtsp:main',
                                          f'watcher = {pkgName}.watcher:main']},
      include_package_data = True,
      install_requires = ['setuptools'],
      name = pkgName,
      packages = [pkgName],
      # package_data = {pkgName: ['someDirectory/*']} ## include files
      package_dir = {pkgName: pkgName},
      version = '0.0.2',
      zip_safe = True)
