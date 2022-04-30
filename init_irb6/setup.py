import setuptools

setuptools.setup(
    name="init_irb6",
    version="1.0",
    author="David Olivier",
    author_email='',
    url='',
    description="Tree trunk detection in images",
    content_type="text/markdown",
    packages=setuptools.find_packages(),
    install_requires=["numpy", "matplotlib", "Phidget22", "gpiozero"]
)