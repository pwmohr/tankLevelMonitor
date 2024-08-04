import sys
import requests
from bs4 import BeautifulSoup
import re
import time

def read_webpage(url):
    try:
        response = requests.get(url)
        if response.status_code == 200:
            s = BeautifulSoup(response.text, 'html.parser')
            t = s.find(id='tankDepth')
            m = re.search('[0-9]+\\.*[0-9]*', t.text)
            return m.group(0)
        else:
            return -1
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    webpage_url = 'http://192.168.0.227'
    while True:
        file_object = open(r"C:\Users\mohr\Downloads\tankData.csv", 'a')
        result = read_webpage(webpage_url)
        humanTime = time.strftime("%d %b %Y, %H:%M:%S", time.localtime())
        machineTime = time.time()
        outString = humanTime + ', ' + str(machineTime) + ', '+ result + ', ' + 'cm\n'
        print(outString)
        file_object.write(outString)
        file_object.close()

        # Sleep for 5 minutes
        time.sleep(5 * 60)