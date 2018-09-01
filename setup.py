from setuptools import setup, find_packages

setup(name='sfdd',
      version='1.0.0',
      description='An implementation of the fault detection method "A sensor-based approach for fault detection and diagnosis for robotic systems"',
      url='https://github.com/youssefmahmoud89/SFDD',
      author='Alex Mitrevski, Youssef Mahmoud Youssef',
      author_email='aleksandar.mitrevski@h-brs.de, youssef-mahmoud.youssef@h-brs.de',
      keywords='sensors fault_detection robotics',
      packages=find_packages(exclude=['contrib', 'docs', 'tests']),
      install_requires=['numpy', 'networkx'],
      project_urls={
          'Method description': 'https://link.springer.com/article/10.1007/s10514-017-9688-z',
          'Source': 'https://github.com/youssefmahmoud89/SFDD'
      })
