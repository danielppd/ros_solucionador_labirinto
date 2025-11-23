from setuptools import find_packages, setup

package_name = 'solucionador_labirinto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inteli',
    maintainer_email='daniel.dias@sou.inteli.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # define o comando 'rodar_solucao' que executa a função main do controlador
            'rodar_solucao = solucionador_labirinto.controlador:main',
        ],
    },
)
