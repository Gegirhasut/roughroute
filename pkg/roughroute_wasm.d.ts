/* tslint:disable */
/* eslint-disable */

/**
 * An offline router holding a loaded road graph in WASM linear memory.
 */
export class WasmRouter {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Load a router from the bytes of a pre-built `.graph` file
     * (a `Uint8Array` over the `ArrayBuffer` the host app fetched from its
     * bundle or cache). Touches no network and no storage.
     *
     * Throws (as a JS error) when the buffer is not a valid `.graph`.
     */
    constructor(graph_bytes: Uint8Array);
    /**
     * Number of nodes in the loaded graph (handy for host-side sanity checks).
     */
    nodeCount(): number;
    /**
     * Route a contract request `{ waypoints: [[lat,lon],…], profile:
     * "car"|"foot" }` to a contract response `{ line, meters, fallback }`.
     *
     * Errors (too few waypoints, waypoint too far from a road, no path with
     * fallback disabled) are thrown as JS errors — the contract itself
     * carries no error field (spec §6.1).
     */
    route(req: any): any;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_wasmrouter_free: (a: number, b: number) => void;
    readonly wasmrouter_new: (a: number, b: number) => [number, number, number];
    readonly wasmrouter_nodeCount: (a: number) => number;
    readonly wasmrouter_route: (a: number, b: any) => [number, number, number];
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __wbindgen_exn_store: (a: number) => void;
    readonly __externref_table_alloc: () => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
