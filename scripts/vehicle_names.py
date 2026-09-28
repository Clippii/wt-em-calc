import json
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'references/vehicle-names/wiki.json'


@lru_cache(maxsize=1)
def naming_sources():
    wiki = json.loads(SOURCE.read_text(encoding='utf-8'))
    records = json.loads((ROOT / 'references/prop-vehicles/manifest.json').read_text())['records']
    return wiki, records


def apply_names(catalog):
    wiki, records = naming_sources()
    vehicles = {r['vehicle'] for r in records}
    for key, row in list(catalog.items()):


        ids = [key] if key in vehicles else sorted(r['vehicle'] for r in records
            if row['propulsion'] in ('jet','rocket') and Path(r['fm'] or '').stem == key)
        labels = []
        for vehicle in ids or [key]:
            name = wiki['names'].get(vehicle)
            labels.append(dict(vehicle_id=vehicle if ids else None,
                               display_name=name,
                               source=wiki['sources'].get(vehicle)))


        visible = [v for v in labels if v['display_name']]
        if not visible:
            del catalog[key]
            continue
        row['vehicle_names'] = visible
        row['name_verified'] = True
        row['name'] = ' / '.join(
            f"{v['display_name']} [{v['vehicle_id'] or key}]" for v in visible)


def refresh_result_names(data, catalog):
    for row in data['aircraft']:
        key = row.get('aircraft_id', row['id'])
        if key not in catalog:
            key = key.removesuffix('__instructor_on').removesuffix('__instructor_off')
        if key not in catalog:
            continue


        suffix = row['name'].partition(' · ')[2]
        row['name'] = catalog[key]['name'] + (' · ' + suffix if suffix else '')
    return data
