from datetime import date
import os
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import xml.etree.ElementTree as ET

website = 'https://dorogoy-dobra.web.app'
filename = 'sitemap.xml'

cred = credentials.Certificate('dd-asia-firebase-adminsdk-ayotk-a6eb56bf4f.json')
firebase_admin.initialize_app(cred)
db = firestore.client()

p = ET.Element('urlset', {
    "xmlns": "http://www.sitemaps.org/schemas/sitemap/0.9"
})

def add_docs(collection, route, changes='daily', priority="0.5"):
    ref = db.collection(collection)
    for doc in ref.stream():
        u = ET.SubElement(p, 'url')
        l = ET.SubElement(u, 'loc')
        l.text = website + '/' + route + '/' + str(doc.id)
        ls = ET.SubElement(u, 'lastmod')
        cf = ET.SubElement(u, 'changefreq')
        cf.text = changes
        pr = ET.SubElement(u, 'priority')
        pr.text = priority

        day = date.today()
        ls.text = f"{day.year}-{'0' + str(day.month) if day.month < 10 else day.month}-{ '0' + str(day.day) if day.day < 10 else day.day}"
    print(f"{collection} written")

def add_route(route, changes='daily', priority="0.5"):
    u = ET.SubElement(p, 'url')
    l = ET.SubElement(u, 'loc')
    l.text = website + '/' + route
    ls = ET.SubElement(u, 'lastmod')
    cf = ET.SubElement(u, 'changefreq')
    cf.text = changes
    pr = ET.SubElement(u, 'priority')
    pr.text = priority

    day = date.today()
    ls.text = f"{day.year}-{'0' + str(day.month) if day.month < 10 else day.month}-{ '0' + str(day.day) if day.day < 10 else day.day}"

add_docs('v2-news', 'article')
add_docs('v2-events', 'event')
add_docs('v2-users', 'user')

add_route('news', priority="0.6")
add_route('calendar', priority="0.6")
add_route('', priority="0.8")

if not os.path.exists(filename):
    with open(filename, 'x'): ...

with open(filename, 'wb') as file:
    file.write(ET.tostring(p, encoding="UTF-8", xml_declaration=True))

print("Sitemap generated")