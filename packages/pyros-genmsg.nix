{
  buildPythonPackage,
  fetchFromGitHub,
  setuptools,
  catkin-pkg,
}:
buildPythonPackage rec {
  pname = "pyros-genmsg";
  version = "0.5.8";
  format = "setuptools";

  src = fetchFromGitHub {
    owner = "ros";
    repo = "genmsg";
    tag = version;
    hash = "sha256-yvT1XVzUhwoFH32EMrrGzrDwe1zCi9ceSIbTakzrbd8=";
  };

  nativeBuildInputs = [
    setuptools
  ];

  propagatedBuildInputs = [
    catkin-pkg
  ];
}
