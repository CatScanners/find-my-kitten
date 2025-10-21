{
  empy,
  fetchPypi,
}:
empy.overrideAttrs (final: old: {
  version = "3.3.4";
  src = fetchPypi {
    inherit (old) pname;
    inherit (final) version;
    hash = "sha256-c6xJeFtgFHnfTqGKfHm8EwSop8NMArlHLPEgauiPAbM=";
  };
})
