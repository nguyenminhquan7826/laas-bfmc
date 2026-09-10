import { HttpClient } from '@angular/common/http';
import { Injectable, NgZone } from '@angular/core';
import { Observable } from 'rxjs';
import { MapMetadata, VehiclesResponse } from './models';

@Injectable({ providedIn: 'root' })
export class MonitoringApiService {
  readonly baseUrl = window.location.port === '4200'
    ? `${window.location.protocol}//${window.location.hostname}:5005`
    : window.location.origin;

  constructor(private readonly http: HttpClient, private readonly zone: NgZone) {}

  vehicles(): Observable<VehiclesResponse> {
    return this.http.get<VehiclesResponse>(`${this.baseUrl}/api/vehicles`);
  }

  map(): Observable<MapMetadata> {
    return this.http.get<MapMetadata>(`${this.baseUrl}/api/map`);
  }

  mapImageUrl(): string {
    return `${this.baseUrl}/api/map/reference`;
  }

  events(onVehicles: (value: VehiclesResponse) => void, onError: () => void): EventSource {
    const stream = new EventSource(`${this.baseUrl}/api/events`);
    stream.addEventListener('vehicles', (event: MessageEvent<string>) => {
      this.zone.run(() => onVehicles(JSON.parse(event.data) as VehiclesResponse));
    });
    stream.onerror = () => this.zone.run(onError);
    return stream;
  }
}
