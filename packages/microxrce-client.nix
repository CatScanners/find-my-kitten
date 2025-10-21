{
  stdenv,
  cmake,
  fetchFromGitHub,
  micro-cdr,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "micro-xrce-dds-client";
  version = "3.0.0";
  src = fetchFromGitHub {
    owner = "eProsima";
    repo = "Micro-XRCE-DDS-Client";
    tag = "v${finalAttrs.version}";
    hash = "sha256-bh9Om36idZ1ybUNn6vHsm6TUDjIccZHTNKgeT8wr+DU=";
  };

  nativeBuildInputs = [
    cmake
  ];

  buildInputs = [
    micro-cdr
  ];
})
