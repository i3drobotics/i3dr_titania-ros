curl 'https://www.alliedvision.com/en/products/software.html?tx_avdownloads_downloads\[action\]=download&tx_avdownloads_downloads\[controller\]=List&cHash=8a61af6c07dfdafa563b4a95458f1082' \
  -H 'Connection: keep-alive' \
  -H 'Cache-Control: max-age=0' \
  -H 'Upgrade-Insecure-Requests: 1' \
  -H 'Origin: https://www.alliedvision.com' \
  -H 'Content-Type: application/x-www-form-urlencoded' \
  -H 'User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/87.0.4280.66 Safari/537.36' \
  -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9' \
  -H 'Sec-Fetch-Site: same-origin' \
  -H 'Sec-Fetch-Mode: navigate' \
  -H 'Sec-Fetch-User: ?1' \
  -H 'Sec-Fetch-Dest: document' \
  -H 'Referer: https://www.alliedvision.com/en/products/software.html' \
  -H 'Accept-Language: en-GB,en-US;q=0.9,en;q=0.8' \
  -H 'Cookie: av_downloads_agb_5496_34=1; av_downloads_agb_5496_33=1; bookmark=%7B%22subModels%22%3A%7B%7D%2C%22cameras%22%3A%5B%5D%2C%22accessories%22%3A%7B%22lenses%22%3A%5B%5D%2C%22interfaceCables%22%3A%5B%5D%2C%22interfaceCards%22%3A%5B%5D%2C%22tripodAdapters%22%3A%5B%5D%2C%22networkings%22%3A%5B%5D%2C%22triggers%22%3A%5B%5D%2C%22powerSupplies%22%3A%5B%5D%7D%7D; _ga=GA1.2.495046925.1606919696; _gid=GA1.2.729625439.1606919696; visitor_id11262=539516763; visitor_id11262-hash=869d844362339d3a026bf81a1e1af50d034a60885c27d43537e51765609866920a23c0d1932f666d7329909b51acfd6c30875e16; _gcl_au=1.1.1537113092.1606919775; _uetsid=bb60e5f034ab11ebaef883f662be79e5; _uetvid=bb6117f034ab11ebbd6b5d64151f85f0; _fbp=fb.1.1606919775406.283803395' \
  --data-raw 'tx_avdownloads_downloads%5B__referrer%5D%5B%40extension%5D=AvDownloads&tx_avdownloads_downloads%5B__referrer%5D%5B%40vendor%5D=Bitmotion&tx_avdownloads_downloads%5B__referrer%5D%5B%40controller%5D=List&tx_avdownloads_downloads%5B__referrer%5D%5B%40action%5D=index&tx_avdownloads_downloads%5B__referrer%5D%5Barguments%5D=YTowOnt9452e261f2a975b936c602cfdd04b9810970c8a19&tx_avdownloads_downloads%5B__referrer%5D%5B%40request%5D=a%3A4%3A%7Bs%3A10%3A%22%40extension%22%3Bs%3A11%3A%22AvDownloads%22%3Bs%3A11%3A%22%40controller%22%3Bs%3A4%3A%22List%22%3Bs%3A7%3A%22%40action%22%3Bs%3A5%3A%22index%22%3Bs%3A7%3A%22%40vendor%22%3Bs%3A9%3A%22Bitmotion%22%3B%7D16b1f73a1c93bc2dfb863fe7edf546efae28eb7f&tx_avdownloads_downloads%5B__trustedProperties%5D=a%3A5%3A%7Bs%3A4%3A%22file%22%3Bi%3A1%3Bs%3A8%3A%22download%22%3Bi%3A1%3Bs%3A6%3A%22plugin%22%3Bi%3A1%3Bs%3A5%3A%22agree%22%3Bi%3A1%3Bs%3A6%3A%22submit%22%3Bi%3A1%3B%7D5808c52e1bdf0ad64a86353c323dc99de425f177&tx_avdownloads_downloads%5Bfile%5D=10620&tx_avdownloads_downloads%5Bdownload%5D=34&tx_avdownloads_downloads%5Bplugin%5D=5496&tx_avdownloads_downloads%5Bagree%5D=' \
  --compressed -o Vimba_v4.2_Linux.tgz
tar -xzf Vimba_v4.2_Linux.tgz
cd Vimba_4_2/VimbaUSBTL/
sudo ./Install.sh
cd ../VimbaGigETL/
sudo ./Install.sh