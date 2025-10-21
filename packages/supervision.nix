{
  buildPythonPackage,
  fetchPypi,

  setuptools,
  pytestCheckHook,
  numpy,
  scipy,
  matplotlib,
  pyyaml,
  defusedxml,
  pillow,
  requests,
  tqdm,
  opencv-python,
}:
buildPythonPackage rec {
  pname = "supervision";
  version = "0.26.1";
  src = fetchPypi {
    inherit pname version;
    hash = "sha256-rw25xUWbtkDPDTHppN8ylgILTNDdSE2GWer+e0dbaPI=";
  };
  pyproject = true;

  build-system = [
    setuptools
  ];

  dependencies = [
    numpy
    scipy
    matplotlib
    pyyaml
    defusedxml
    pillow
    requests
    tqdm
    opencv-python
  ];

  pythonImportsCheck = [
    "supervision"
  ];
}
