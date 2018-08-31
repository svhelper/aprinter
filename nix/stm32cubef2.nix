/*
 * Copyright (c) 2015 Ambroz Bizjak
 * Copyright (c) 2018 BOBAH
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

{ stdenv, fetchurl, unzip }:
let
    source = fetchurl {
        url = http://www.st.com/resource/en/firmware2/stm32cube_fw_f2_v170.zip;
        sha256 = "2F7A2822BAF14150B662C9649ECD9F8333667A86C7AE1C924349C68AB58D58FE";
    };
    patchFiles = [ ../patches/stm32cubef2.patch ../patches/stm32cubef2-warnings.patch ];
in
stdenv.mkDerivation rec {
    name = "stm32cubef2";
    
    unpackPhase = "true";
    
    nativeBuildInputs = [ unzip ];
    
    installPhase = ''
        mkdir -p "$out"/EXTRACT
        unzip -q ${source} -d "$out"/EXTRACT
        mv "$out"/EXTRACT/STM32Cube*/* "$out"/
        rm -rf "$out"/EXTRACT
        ${stdenv.lib.concatStrings (map (patchFile: "patch -d \"$out\" -p1 < ${patchFile}\n") patchFiles)}
    '';
    
    dontStrip = true;
    dontPatchELF = true;
    dontPatchShebangs = true;
}
