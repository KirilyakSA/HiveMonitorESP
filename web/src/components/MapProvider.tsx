import { useEffect, useRef } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import type { Apiary } from "../api";

type LatLng = { lat: number; lng: number };

export type MapProviderName = "openstreetmap";

type ApiaryMapProps = {
  apiaries: Apiary[];
  selectedApiaryId?: string;
  provider?: MapProviderName;
  pickMode?: boolean;
  pickedPosition?: LatLng | null;
  onSelectApiary?: (apiaryId: string) => void;
  onPick?: (position: LatLng) => void;
};

export function ApiaryMap({
  apiaries,
  selectedApiaryId,
  provider = "openstreetmap",
  pickMode = false,
  pickedPosition,
  onSelectApiary,
  onPick
}: ApiaryMapProps) {
  if (provider !== "openstreetmap") {
    return <div className="map-unavailable">Провайдер карты пока не подключен</div>;
  }

  return (
    <OpenStreetMap
      apiaries={apiaries}
      selectedApiaryId={selectedApiaryId}
      pickMode={pickMode}
      pickedPosition={pickedPosition}
      onSelectApiary={onSelectApiary}
      onPick={onPick}
    />
  );
}

function OpenStreetMap({ apiaries, selectedApiaryId, pickMode, pickedPosition, onSelectApiary, onPick }: Omit<ApiaryMapProps, "provider">) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const apiaryLayerRef = useRef<L.LayerGroup | null>(null);
  const pickLayerRef = useRef<L.LayerGroup | null>(null);
  const onPickRef = useRef(onPick);

  useEffect(() => {
    onPickRef.current = onPick;
  }, [onPick]);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const center = firstApiaryPoint(apiaries) ?? { lat: 50.45, lng: 30.52 };
    const map = L.map(containerRef.current, { zoomControl: true }).setView([center.lat, center.lng], firstApiaryPoint(apiaries) ? 9 : 6);
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }).addTo(map);
    apiaryLayerRef.current = L.layerGroup().addTo(map);
    pickLayerRef.current = L.layerGroup().addTo(map);
    map.on("click", (event: L.LeafletMouseEvent) => {
      if (!onPickRef.current) return;
      onPickRef.current({ lat: event.latlng.lat, lng: event.latlng.lng });
    });
    mapRef.current = map;

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  useEffect(() => {
    const map = mapRef.current;
    const layer = apiaryLayerRef.current;
    if (!map || !layer) return;

    layer.clearLayers();
    const points: LatLng[] = [];

    apiaries.forEach((apiary) => {
      if (!isGeoApiary(apiary)) return;
      points.push({ lat: apiary.latitude, lng: apiary.longitude });
      const selected = apiary.id === selectedApiaryId;
      const marker = L.circleMarker([apiary.latitude, apiary.longitude], {
        radius: selected ? 10 : 7,
        color: selected ? "#f5a600" : "#2563eb",
        weight: selected ? 3 : 2,
        fillColor: selected ? "#fbbf24" : "#3b82f6",
        fillOpacity: 0.85
      }).addTo(layer);
      marker.bindTooltip(apiary.name, { direction: "top", offset: [0, -8] });
      marker.on("click", () => onSelectApiary?.(apiary.id));
    });

    if (points.length > 0) {
      const bounds = L.latLngBounds(points.map((point) => [point.lat, point.lng]));
      map.fitBounds(bounds.pad(0.25), { maxZoom: 12 });
    }
  }, [apiaries, selectedApiaryId, onSelectApiary]);

  useEffect(() => {
    const layer = pickLayerRef.current;
    if (!layer) return;
    layer.clearLayers();
    if (!pickedPosition) return;

    L.circleMarker([pickedPosition.lat, pickedPosition.lng], {
      radius: 8,
      color: "#f5a600",
      weight: 3,
      fillColor: "#fbbf24",
      fillOpacity: 0.85
    }).addTo(layer);
    mapRef.current?.setView([pickedPosition.lat, pickedPosition.lng], Math.max(mapRef.current.getZoom(), 12));
  }, [pickedPosition]);

  return <div ref={containerRef} className={`osm-map ${pickMode ? "pick-mode" : ""}`} />;
}

function firstApiaryPoint(apiaries: Apiary[]) {
  const apiary = apiaries.find(isGeoApiary);
  return apiary ? { lat: apiary.latitude, lng: apiary.longitude } : null;
}

function isGeoApiary(apiary: Apiary): apiary is Apiary & { latitude: number; longitude: number } {
  return typeof apiary.latitude === "number" && typeof apiary.longitude === "number";
}
