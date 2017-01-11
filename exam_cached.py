import json

selected_prms = [
    ('4',2.0,2.0,10.0,2.0),
    ('4',2.0,2.0,50.0,40.0)
]


hashed_prms = [ str(hash(xx)) for xx in selected_prms ]

selected = {}
with open('cached.json') as rr:
    cached = json.loads(rr.read())
    print cached.keys()
    for hashed_prm in hashed_prms:
        if hashed_prm in cached:
	    print 'found ', hashed_prm
        selected[hashed_prm] = cached[hashed_prm]
with open('cached.json','w+') as ww:
    ww.write(json.dumps(selected))



