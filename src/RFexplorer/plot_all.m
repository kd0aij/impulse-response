basePath = "/home/markw/Desktop/RFexplorer/maxhold2"

for i=[1:13]
  plotRF(basePath, i)
  input(':')
endfor
