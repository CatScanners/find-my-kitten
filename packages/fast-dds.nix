{
  stdenv,
  cmake,
  fetchFromGitHub,
  fetchpatch2,
  asio,
  foonathan-memory,
  tinyxml-2,
  fast-cdr,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "fast-dds";
  version = "3.1.0";
  src = fetchFromGitHub {
    owner = "eProsima";
    repo = "Fast-DDS";
    tag = "v${finalAttrs.version}";
    hash = "sha256-YUAIuIQapa+SzYKE+/GFYMI4tjBrldmojHkYHim0mFw=";
  };

  patches = [
    (fetchpatch2 { # https://github.com/eProsima/Fast-DDS/pull/5341
      url = "https://github.com/eProsima/Fast-DDS/commit/81f00693974dc603bd67e1afcb0ae85f273071f6.patch";
      hash = "sha256-gPN1XUvYqCa4MrQ3Y7mzgcsfagSG7znph340CyeIE5o=";
    })
  ];

  enableParallelBuilding = true;

  nativeBuildInputs = [
    cmake
  ];

  buildInputs = [
    fast-cdr
    asio
    foonathan-memory
  ];

  propagatedBuildInputs = [
    tinyxml-2
  ];
})
