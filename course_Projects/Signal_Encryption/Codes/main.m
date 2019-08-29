clc;
clear all;

import javax.crypto.Cipher;


inpImg = (imread('The Solvay Conference, probably the most intelligent picture ever taken, 1927 (2).jpg'));
s = size(inpImg); % Find the size of image Vector (x and y)
%convert data to vector for RSA javax Format
inpImg = inpImg(:);
%plot(inpImg)

%RSA Key Generation
cipher = Cipher.getInstance('RSA');
keyGen = java.security.KeyPairGenerator.getInstance('RSA');
keyPair = keyGen.genKeyPair();
priv = keyPair.getPrivate(); %generate private key
modn = keyPair.getPublic(); %generate public key and 1024-bit modulus key
enImg = [];
%Encryption
cipher.init(Cipher.ENCRYPT_MODE, priv);
for u=1:numel(inpImg)
    plaintextUnicodeVals = uint16(inpImg(u));
    temp = typecast(plaintextUnicodeVals, 'int8');
    enImg(u) = cipher.doFinal(temp(u));
end




