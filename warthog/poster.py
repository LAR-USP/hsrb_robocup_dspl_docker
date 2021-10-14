import requests

url = 'http://172.19.0.1:5000/countries'
# url = 'http://0.0.0.0:5000/countries'
# url = 'http://172.17.0.1:5000/countries'
# url = 'http://172.17.0.1:5000/countries'
myobj = {"name":"Germany", "capital": "Berlin", "area": 357022}

x = requests.post(url, json = myobj)

print(x.text)